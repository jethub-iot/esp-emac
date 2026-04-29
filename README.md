# esp-emac

[![License: GPL-2.0-or-later OR Apache-2.0](https://img.shields.io/badge/license-GPL--2.0--or--later%20OR%20Apache--2.0-blue.svg)](#license)

Native ESP32 Ethernet MAC driver for `#![no_std]` Rust. Owns the DMA
engine and brings the EMAC peripheral up directly via memory-mapped
register helpers — no `ph-esp32-mac`, no `esp-idf-svc`, no
`esp-eth`.

Targets the **built-in EMAC** on the original ESP32 (Xtensa, dual-core).
ESP32-S3 / C6 / H2 don't have an EMAC peripheral; for those use SPI
Ethernet (e.g. W5500 / ENC28J60). The crate compiles only when the
target is `xtensa-esp32-none-elf`.

## What's in the box

* `Emac<RX, TX, BUF>` — the driver. RX/TX descriptor ring sizes and
  per-buffer length are const generics so the entire packet memory
  layout is static; nothing on the heap.
* `EmacDriver` — `embassy_net_driver::Driver` adaptor (feature
  `embassy-net`). Tokens borrow into a fixed buffer, no allocations.
* `EspMdio` — Station Management (SMI / MDIO) controller. Implements
  `eth_mdio_phy::MdioBus` (feature `mdio-phy`) so any PHY driver
  written against that trait Just Works.
* `regs::{mac, dma, ext, gpio}` — typed bit constants + tiny `read` /
  `write` / `set_bits` / `clear_bits` helpers + composite operations
  (`set_mac_address`, `start_tx`, `enable_peripheral_clock`, ...).
  Use these directly only if you need to do something the high-level
  `Emac` API doesn't expose.
* `reset::ResetController` — DMA software-reset state machine; takes
  any `embedded_hal::delay::DelayNs`.
* `clock` — APLL 50 MHz programming for the RMII reference clock,
  plus GPIO0/16/17 routing.

## Quick start (bare metal + LAN8720A)

```rust no_run
use esp_hal::{delay::Delay, rng::Rng};
use esp_emac::{
    Emac, EmacConfig, RmiiClockConfig, RmiiPins, ClkGpio,
};
use esp_emac::mdio::EspMdio;
use eth_phy_lan87xx::PhyLan87xx;
use eth_mdio_phy::PhyDriver;

// 1. Static state — must outlive the driver because DMA holds raw
//    pointers into it. const generics fix the buffer sizes.
static mut EMAC: Emac<10, 10, 1600> = Emac::new(EmacConfig {
    clock: RmiiClockConfig::InternalApll { gpio: ClkGpio::Gpio17 },
    pins: RmiiPins { mdc: 23, mdio: 18 },
});

# fn example() -> Result<(), esp_emac::EmacError> {
# let mut delay = Delay::new();
// 2. Bring it up.
let emac = unsafe { &mut *core::ptr::addr_of_mut!(EMAC) };
emac.set_mac_address([0x00, 0x70, 0x07, 0x24, 0x3B, 0x87]);
emac.init(&mut delay)?;

// 3. Talk to the PHY through SMI.
let mut mdio = EspMdio::new();
let mut phy = PhyLan87xx::new(/* PHY addr */ 1);
phy.init(&mut mdio).expect("PHY init");

// 4. Wait for link, programme speed/duplex, start.
if let Ok(Some(status)) = phy.poll_link(&mut mdio) {
    emac.set_speed(status.speed.into());
    emac.set_duplex(status.duplex.into());
}
emac.start()?;

// 5. From here either use embassy-net (feature `embassy-net`) or
//    the synchronous `emac.transmit(&buf)` / `emac.receive(&mut buf)`
//    API directly.
# Ok(())
# }
```

For the embassy-net path see [`embassy::EmacDriver`](src/embassy.rs)
and the live firmware integration in
[`testsystem-firmware-esp / src-hal/firmware/src/net/ethernet.rs`](https://github.com/jethome-iot/testsystem-firmware-esp/blob/main/src-hal/firmware/src/net/ethernet.rs).

## Hardware bring-up sequence

`Emac::init` follows the canonical ESP32 GMAC sequence — every step is
documented inline at [`src/emac.rs`](src/emac.rs):

1. Programme APLL to 50 MHz and route the RMII clock to the chosen
   GPIO (`InternalApll`) — or set GPIO0 IO_MUX function 5 to take an
   external 50 MHz oscillator (`External`).
2. Configure SMI pins (MDC=GPIO23, MDIO=GPIO18 by default) through the
   GPIO Matrix; route the six fixed RMII data pins
   (TXD0=19, TXD1=22, TX_EN=21, RXD0=25, RXD1=26, CRS_DV=27) through
   IO_MUX function 5.
3. Enable the EMAC peripheral clock via DPORT.
4. Set the PHY interface (RMII) and the chosen clock source.
5. Enable the EMAC extension clocks and power up the EMAC RAM.
6. Issue a DMA software reset; wait for `DMABUSMODE.SWR` to self-clear.
7. Programme the MAC core: PORT_SELECT=1 (MII/RMII), 100 Mbps, full
   duplex, auto-pad/CRC strip, jabber/watchdog disabled. Frame filter
   passes broadcast + all multicast; perfect-match unicast filter is
   on `ADDR0`.
8. Programme the DMA bus mode (`ATDS=1` enhanced 8-word descriptors,
   PBL=32, AAL, USP, FIXED_BURST) and operation mode
   (TSF + RSF — store-and-forward).
9. Hand the DMA the descriptor list base addresses.
10. Programme the primary MAC address into `GMACADDR0H/L` —
    **HIGH first, then LOW**. The Synopsys GMAC core latches the
    filter address on the LOW write; doing it the other way around
    leaves the internal latch holding the stale reset value (unicast
    RX silently dies, register read-back lies).

`Emac::start` then enables MAC TX, DMA TX, DMA RX, MAC RX in that order
and issues a poll-demand to wake the RX DMA out of `Suspended`.

## Choosing static buffer sizes

`Emac<RX, TX, BUF>` is const-generic on the RX/TX ring counts and the
per-buffer length. Each descriptor is 32 bytes (ATDS layout); each
buffer is `BUF` bytes (typical 1536 or 1600).

| Profile       | RX | TX | BUF  | RAM     |
|---------------|----|----|------|---------|
| `EmacDefault` | 10 | 10 | 1600 | ~32 KiB |
| `EmacSmall`   |  4 |  4 | 1600 | ~13 KiB |

`Emac::memory_usage()` returns the exact byte count for any chosen
combination. Pick the size at compile time; the value lives in `.bss`.

## Cargo features

| Feature       | Default | Pulls in                                        |
|---------------|---------|-------------------------------------------------|
| `esp-hal`     | off     | `esp_hal::interrupt::*` for ISR binding         |
| `mdio-phy`    | off     | `eth-mdio-phy` (and an `EspMdio: MdioBus` impl) |
| `embassy-net` | off     | `embassy-net-driver` + `embassy-sync`           |
| `defmt`       | off     | `defmt::Format` derives on public types         |

The firmware build typically enables `esp-hal` + `mdio-phy` +
`embassy-net` together.

## Known gotchas

These are baked into the driver but worth knowing if you're reading
the source or porting elsewhere:

* **`GMACADDR0` write order.** HIGH first (with the `AE` bit at
  bit 31), LOW second. The internal address-filter latch fires on the
  LOW write only. — `regs::mac::set_mac_address`.
* **`DMARXPOLLDEMAND` after every successful `receive()`.** Without
  it the RX DMA enters `Suspended` once the ring drains and never
  recovers, even though the descriptor that just freed up is now
  CPU-owned. — `Emac::receive`.
* **RX descriptor `ATDS=1`.** The MAC writes RX status into descriptor
  word 4, which only exists in the enhanced 8-word layout
  (`DMABUSMODE.ATDS=1`). Stick to the `dma::descriptor` types in
  this crate — the legacy 4-word layout silently mis-decodes
  every received frame.
* **APLL 50 MHz must be programmed BEFORE the DMA software reset.**
  The reset sequencer needs a working RMII reference clock to
  deassert the busy bit.
* **PHY reset register.** A `BMCR.RESET` cycle does NOT restore
  `ANAR` to `0x01E1` on the LAN8720A on cold boot. After resetting
  the PHY, write `0x01E1` to `ANAR` explicitly before kicking
  auto-negotiation. Already handled in `eth-phy-lan87xx`.

## Hardware verified on

* JXD-PM3-80-E1ETH (factory MAC, BLK3 efuse empty)
* JXD-R6-E1ETH-LCD (custom MAC `f0:57:8d:01:04:e0` programmed in BLK3
  via `espefuse.py burn_custom_mac`)

Cold boot, soft reset (DTR-toggle / `RTC_CNTL.SW_SYS_RST`), and USB
power-cycle all yield the same behaviour: PHY init → link up
100 Mbps full → DHCP → ICMP/HTTP.

## License

Licensed under either of:

* GNU General Public License, Version 2.0 or later
  ([LICENSE-GPL](LICENSE-GPL))
* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))

at your option.

Copyright (c) Viacheslav Bocharov (v at baodeep dot com) and JetHome (r).
