# esp-emac

[![License: GPL-2.0-or-later OR Apache-2.0](https://img.shields.io/badge/license-GPL--2.0--or--later%20OR%20Apache--2.0-blue.svg)](#license)
[![Crates.io](https://img.shields.io/crates/v/esp-emac.svg)](https://crates.io/crates/esp-emac)
[![Documentation](https://docs.rs/esp-emac/badge.svg)](https://docs.rs/esp-emac)

Native ESP32 Ethernet MAC driver for `#![no_std]` Rust. Owns the DMA
engine and brings the EMAC peripheral up directly via memory-mapped
register helpers — no `ph-esp32-mac`, no `esp-idf-svc`, no `esp-eth`.

Pairs with [`eth-phy-lan87xx`](https://crates.io/crates/eth-phy-lan87xx)
(or any [`eth_mdio_phy::PhyDriver`](https://docs.rs/eth-mdio-phy)
implementation) for the PHY side, and with `embassy-net` for the
TCP/IP stack.

---

## Installation

```toml
[dependencies]
esp-emac        = { version = "0.2", features = ["esp-hal", "mdio-phy", "embassy-net"] }
eth-mdio-phy    = "0.2"
eth-phy-lan87xx = "0.2"   # or any other eth_mdio_phy::PhyDriver impl

# Required runtime stack
esp-hal           = { version = "1.1", features = ["esp32", "unstable"] }
embassy-executor  = "0.10"
embassy-net       = { version = "0.9", features = ["dhcpv4", "medium-ethernet"] }
embassy-time      = "0.5"
static_cell       = "2"
embedded-hal      = "1.0"
esp-backtrace     = { version = "0.19", features = ["esp32", "panic-handler", "println"] }
esp-println       = { version = "0.17", default-features = false, features = ["esp32", "uart"] }
esp-rtos          = { version = "0.3", features = ["esp32", "embassy"] }
```

Target triple: `xtensa-esp32-none-elf` (install via `espup install`).
**MSRV: 1.88** (constrained by `esp-hal = "1.1"`'s declared
`rust-version`). The driver works only on the original ESP32
(Xtensa LX6).

### Features

| Feature | Default | Pulls in | When to enable |
| --- | --- | --- | --- |
| `esp-hal` | off | `esp_hal::interrupt` for ISR binding | Always, for hardware bring-up |
| `mdio-phy` | off | `eth-mdio-phy` (and `EspMdio: MdioBus` impl) | When using a `PhyDriver`-based PHY (LAN87xx etc.) |
| `embassy-net` | off | `embassy-net-driver`, `embassy-sync`, `critical-section` | When using `embassy-net` TCP/IP stack |
| `async` | off | `embedded-hal-async` | When using `AsyncResetController` |
| `defmt` | off | `defmt::Format` derives | When logging through `defmt` |

The typical firmware build enables `esp-hal + mdio-phy + embassy-net`.

### Compatibility

| esp-emac | esp-hal | embassy-net | embassy-executor | Rust target |
| --- | --- | --- | --- | --- |
| 0.2.x | 1.1.x | 0.9.x | 0.10.x | `xtensa-esp32-none-elf` |

Other ESP variants (S2/S3/C-series/H2) have **no** built-in EMAC — use
SPI Ethernet (W5500, ENC28J60) instead. ESP32-P4 has a newer Synopsys
GMAC revision and is not yet supported (planned).

---

## Quick start (embassy-net + LAN8720A)

The complete working example is in
[`examples/embassy_net_lan8720a.rs`](examples/embassy_net_lan8720a.rs).
The skeleton looks like this:

```rust no_run
#![no_std]
#![no_main]

use esp_backtrace as _; // installs the `#[panic_handler]`

use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_hal::delay::DelayNs;
use esp_hal::{delay::Delay, interrupt::Priority, rng::Rng};

use esp_emac::config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins, XtalFreq};
use esp_emac::emac::{Duplex as EmacDuplex, Speed as EmacSpeed};
use esp_emac::embassy::{EmacDefaultDriver, EmacDriverState};
use esp_emac::mdio::EspMdio;
use esp_emac::EmacDefault;

use eth_mdio_phy::{Duplex as PhyDuplex, PhyDriver, Speed as PhySpeed};
use eth_phy_lan87xx::PhyLan87xx;

// 1. Static storage — DMA holds raw pointers into the `Emac` instance,
//    so it must live in `static` and never move. `Emac::new` (and
//    therefore `EmacDefault::new`) is a `const fn`, so the value is
//    built at compile time and lives in BSS — zero runtime stack cost
//    on boot. The default ring sizing is currently 10 RX / 10 TX /
//    1600-byte buffers (~32 KiB), sourced from `DEFAULT_RX` /
//    `DEFAULT_TX` / `DEFAULT_BUF`. We deliberately do NOT wrap it in
//    `StaticCell::init(EmacDefault::new(...))` — that pattern would
//    risk materialising the 32 KiB struct on the caller's stack
//    before moving it into the cell. The `static mut` form below
//    avoids that hazard at the cost of one well-isolated `unsafe`.
static mut EMAC: EmacDefault = EmacDefault::new(EmacConfig {
    clock: RmiiClockConfig::InternalApll {
        gpio: ClkGpio::Gpio17,
        xtal: XtalFreq::Mhz40,
    },
    pins: RmiiPins { mdc: 23, mdio: 18 },
});
static EMAC_STATE: EmacDriverState = EmacDriverState::new();

// 2. Bind the EMAC interrupt to the driver's state.
#[esp_hal::handler(priority = Priority::Priority1)]
fn emac_interrupt_handler() {
    EMAC_STATE.handle_emac_interrupt();
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, EmacDefaultDriver<'static>>) {
    runner.run().await
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // esp-rtos owns the embassy timer + scheduler. Start it before any
    // `Timer::after(...)` or `spawner.spawn(...)` can fire.
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let mut delay = Delay::new();
    let rng = Rng::new();

    // 3. Bring up MAC + PHY. SAFETY: EMAC is touched only here — single
    //    owner — so no aliasing.
    let emac = unsafe { &mut *core::ptr::addr_of_mut!(EMAC) };
    emac.set_mac_address([0x00, 0x70, 0x07, 0x24, 0x3B, 0x87]);
    emac.init(&mut delay).expect("EMAC init");
    emac.bind_interrupt(emac_interrupt_handler);

    let mut mdio = EspMdio::new();
    let mut phy = PhyLan87xx::new(/* PHY addr */ 1);
    phy.init(&mut mdio).expect("PHY init");

    // 4. Wait for link, programme speed/duplex.
    loop {
        match phy.poll_link(&mut mdio) {
            Ok(Some(status)) => {
                emac.set_speed(match status.speed {
                    PhySpeed::Mbps10 => EmacSpeed::Mbps10,
                    PhySpeed::Mbps100 => EmacSpeed::Mbps100,
                });
                emac.set_duplex(match status.duplex {
                    PhyDuplex::Half => EmacDuplex::Half,
                    PhyDuplex::Full => EmacDuplex::Full,
                });
                EMAC_STATE.set_link_up();
                break;
            }
            Ok(None) => delay.delay_ms(200),
            Err(_) => delay.delay_ms(200),
        }
    }

    emac.start().expect("EMAC start");

    // 5. Plumb into embassy-net. `EmacDefaultDriver` is a type alias
    //    whose inherent `new` is `EmacDriver::new` — keeps the call
    //    site free of the const-generic ceremony (currently
    //    `<10, 10, 1600>`, sourced from `DEFAULT_RX` / `DEFAULT_TX` /
    //    `DEFAULT_BUF`).
    let driver = EmacDefaultDriver::new(emac, &EMAC_STATE);
    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);

    static RESOURCES: static_cell::StaticCell<StackResources<8>> =
        static_cell::StaticCell::new();
    let (stack, runner) = embassy_net::new(
        driver,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        RESOURCES.init(StackResources::<8>::new()),
        net_seed,
    );

    spawner.spawn(net_task(runner)).unwrap();

    // 6. Wait for DHCP, use the stack.
    loop {
        if let Some(cfg) = stack.config_v4() {
            // got IP address: cfg.address
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}
```

Bare-metal sync usage (without `embassy-net`) is documented in the
crate-level rustdoc — see [`Emac::transmit`](https://docs.rs/esp-emac/latest/esp_emac/emac/struct.Emac.html#method.transmit)
and [`Emac::receive`](https://docs.rs/esp-emac/latest/esp_emac/emac/struct.Emac.html#method.receive).

---

## Interrupt binding

`EmacDriver` is event-driven: each frame received or descriptor freed
fires a MAC interrupt that wakes the embassy-net runner. Three pieces
need to line up:

1. **A `static EMAC_STATE: EmacDriverState`.** Holds the `WAKER` the
   driver polls and the `link_up` flag. Created with
   `EmacDriverState::new()` and never moved.
2. **A handler annotated with `#[esp_hal::handler]`** that calls
   `EMAC_STATE.handle_emac_interrupt()`. Use `Priority::Priority1` —
   the driver does not gate on priority, but level 1 keeps it well below
   timer/scheduler interrupts.
3. **`emac.bind_interrupt(handler)`** after `init()` — this maps the
   ESP32 EMAC IRQ to the handler symbol via `esp-hal`'s interrupt
   table.

Forgetting step 3 silently produces a working link but no incoming
frames at the embassy-net layer (`is_link_up()` true, `config_v4()`
permanently `None`).

---

## Troubleshooting

### Link is up but DHCP never completes

Symptoms: `stack.is_link_up()` returns `true`, but `stack.config_v4()`
stays `None` for tens of seconds.

Most likely:

* **MAC address bit 0 set** (multicast bit). The frame filter rejects
  multicast as a source — the DHCP server's reply is delivered but
  silently dropped before user space. Double-check the bytes you pass
  to `set_mac_address`.
* **Interrupt handler not bound** (see [Interrupt binding](#interrupt-binding)).
* **PHY ANAR not restored after cold boot.** Use `eth-phy-lan87xx`
  (which writes `ANAR=0x01E1` explicitly) or follow that pattern in your
  custom PHY driver.

### `EmacError::InvalidConfig` on `init`

You picked an impossible `RmiiClockConfig`:

* `External { Gpio16 / Gpio17 }` — those pads only have an output
  function 5 on ESP32. Only `Gpio0` works as RMII clock input.
* `InternalApll { Gpio0 }` — `Gpio0` only has the input function on
  this peripheral. Use `Gpio16` (0° phase) or `Gpio17` (180° phase).

### Link goes up at 10 Mbps when the PHY supports 100 Mbps

`ANAR` got partially programmed and auto-neg converged on a subset.
Cold boot of LAN87xx is the textbook case — that's why `eth-phy-lan87xx`
writes `ANAR=0x01E1` explicitly. If using a different PHY driver, mirror
that pattern.

### Unicast RX silently fails (broadcast/multicast still arrive)

The MAC address-filter latch in `GMACADDR0` was programmed in the
wrong order. **HIGH first, LOW second** — the latch fires on the LOW
write. `Emac::set_mac_address` does this correctly; if you bypass it
and write through `regs::mac::*` raw, observe the order and the
`AE` (`ADDRESS_ENABLE`) bit at bit 31 of `GMACADDR0H`.

### `XtalFreq::Mhz40` but the link still won't come up

Verify your module's actual crystal — there is no runtime detection.
Most ESP32 modules (WROOM, WROVER, MINI, JXD-CPU-E1ETH) ship with
40 MHz, but some legacy boards have 26 MHz. Picking the wrong value
silently produces an off-frequency RMII reference clock.

---

## Reference

### What's in the box

* `Emac<RX, TX, BUF>` — the driver. RX/TX descriptor ring sizes and
  per-buffer length are const generics so the entire packet memory
  layout is static; nothing on the heap.
* `EmacDriver` — `embassy_net_driver::Driver` adaptor (feature
  `embassy-net`). Tokens copy frames through a stack-allocated buffer
  on every `consume`; no heap allocations.
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

### RMII clock modes

ESP32 supports two mutually exclusive RMII reference-clock modes. The
choice is dictated by the board layout — `Emac::init` rejects mismatched
GPIO selections with `EmacError::InvalidConfig`.

| Mode | GPIO | Direction | When to use | Caveat |
| --- | --- | --- | --- | --- |
| `InternalApll { Gpio16, xtal }` | 16 | output (`EMAC_CLK_OUT`, 0°) | dev boards where the MCU drives the PHY's REF_CLK pin | Errata CLK-3.22 — clock pad is corrupted by RF noise during WiFi/BT TX. Avoid if the radio is active. |
| `InternalApll { Gpio17, xtal }` | 17 | output (`EMAC_CLK_OUT_180`, 180°) | LAN8720A reference design — phase shift improves RX setup margin | Same CLK-3.22 caveat. |
| `External { Gpio0 }` | 0 | input (`EMAC_TX_CLK`) | production designs with a PHY crystal / oscillator (e.g. JXD-CPU-E1ETH); required for Ethernet + WiFi coexistence | GPIO0 is also the boot-strapping pin — make sure the oscillator level at reset matches the boot-mode requirement. |

`xtal` is an [`XtalFreq`](src/config.rs) enum (`Mhz26`, `Mhz32`, `Mhz40`)
selecting APLL SDM coefficients. **It must match the actual on-board
crystal** — there is no detection at runtime.

### Hardware bring-up sequence

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
    **HIGH first, then LOW**.

`Emac::start` then enables MAC TX, DMA TX, DMA RX, MAC RX in that order
and issues a poll-demand to wake the RX DMA out of `Suspended`.

### Choosing static buffer sizes

`Emac<RX, TX, BUF>` is const-generic on the RX/TX ring counts and the
per-buffer length. Each descriptor is 32 bytes (ATDS layout); each
buffer is `BUF` bytes (typical 1536 or 1600).

| Profile | RX | TX | BUF | RAM |
| --- | --- | --- | --- | --- |
| `EmacDefault` | 10 | 10 | 1600 | ~32 KiB |
| `EmacSmall` | 4 | 4 | 1600 | ~13 KiB |

`Emac::memory_usage()` returns the exact byte count for any chosen
combination. Pick the size at compile time; the value lives in `.bss`.

> **`Emac::default()` is intentionally not provided.** The clock and
> pin configuration is hardware-specific and any default the crate
> could pick (internal APLL on GPIO17, MDC/MDIO 23/18) would silently
> mis-drive boards that expect a different layout. Always construct an
> explicit `EmacConfig`.

### Known gotchas (baked into the driver)

* **`GMACADDR0` write order.** HIGH first (with the `AE` bit at
  bit 31), LOW second. The internal address-filter latch fires on the
  LOW write only. — `regs::mac::set_mac_address`.
* **`DMARXPOLLDEMAND` after every successful `receive()`.** Without
  it the RX DMA enters `Suspended` once the ring drains and never
  recovers. — `Emac::receive`.
* **RX descriptor `ATDS=1`.** The MAC writes RX status into descriptor
  word 4, which only exists in the enhanced 8-word layout. The legacy
  4-word layout silently mis-decodes every received frame.
* **APLL 50 MHz must be programmed BEFORE the DMA software reset.**
  The reset sequencer needs a working RMII reference clock to
  deassert the busy bit.
* **PHY reset register.** A `BMCR.RESET` cycle does NOT restore
  `ANAR` to `0x01E1` on the LAN8720A on cold boot. After resetting
  the PHY, write `0x01E1` to `ANAR` explicitly. Already handled in
  `eth-phy-lan87xx`.

---

## Hardware verified on

* JXD-PM3-80-E1ETH (factory MAC, BLK3 efuse empty)
* JXD-R6-E1ETH-LCD (custom MAC `f0:57:8d:01:04:e0` programmed in BLK3
  via `espefuse.py burn_custom_mac`)

Cold boot, soft reset (DTR-toggle / `RTC_CNTL.SW_SYS_RST`), and USB
power-cycle all yield the same behaviour: PHY init → link up
100 Mbps full → DHCP → ICMP/HTTP.

A reference firmware integration is in
[`testsystem-firmware-esp / src-hal/firmware/src/net/ethernet.rs`](https://github.com/jethome-iot/testsystem-firmware-esp/blob/main/src-hal/firmware/src/net/ethernet.rs).

## License

Licensed under either of:

* GNU General Public License, Version 2.0 or later
  ([LICENSE-GPL](LICENSE-GPL))
* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))

at your option.

Copyright (c) Viacheslav Bocharov (v at baodeep dot com) and JetHome (r).
