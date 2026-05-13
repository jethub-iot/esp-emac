//! Embassy-net + LAN8720A example for ESP32 with built-in EMAC.
//!
//! Brings up the MAC + PHY, joins DHCP and prints the acquired
//! address. Build with:
//!
//! ```sh
//! cargo build --release --example embassy_net_lan8720a \
//!   --features esp-hal,mdio-phy,embassy-net \
//!   --target xtensa-esp32-none-elf
//! ```
//!
//! Hardware assumptions:
//!
//! - LAN8720A on the standard ESP32 RMII pinout
//!   (TXD0=19, TXD1=22, TX_EN=21, RXD0=25, RXD1=26, CRS_DV=27,
//!   MDC=23, MDIO=18, REF_CLK on GPIO17).
//! - 40 MHz crystal on the ESP32 module (WROOM/WROVER/MINI).
//! - PHY answers MDIO on address 1.
//!
//! Adjust [`MAC_ADDRESS`], [`RmiiPins`], the PHY address and the
//! clock mode if your board differs.

#![no_std]
#![no_main]

use esp_backtrace as _; // installs the `#[panic_handler]`

use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::{
    delay::Delay,
    interrupt::{software::SoftwareInterruptControl, Priority},
    rng::Rng,
};

use esp_emac::config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins, XtalFreq};
use esp_emac::embassy_net::{EmacDefaultDriver, EmacDriverState};
use esp_emac::mdio::EspMdio;
use esp_emac::EmacDefault;

use eth_mdio_phy::PhyDriver;
use eth_phy_lan87xx::PhyLan87xx;

use static_cell::StaticCell;

/// Locally-administered unicast MAC. Replace with one from your eFuse
/// or OUI block.
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

/// MDIO address of the PHY (strap pins on LAN8720A typically pull this
/// to 0 or 1).
const PHY_ADDR: u8 = 1;

/// EMAC instance — DMA holds raw pointers into this, so it must live in
/// `static` storage and never move. `EmacDefault::new` is a `const fn`
/// so the value is built at compile time and lives in BSS; no runtime
/// stack temporary on boot. The default ring sizing is ~32 KiB
/// (`DEFAULT_RX * DEFAULT_BUF` + `DEFAULT_TX * DEFAULT_BUF` plus
/// descriptor rings) —
/// using `StaticCell::init(EmacDefault::new(..))` instead would risk
/// landing that whole struct on the calling task's stack frame.
static mut EMAC: EmacDefault = EmacDefault::new(EmacConfig {
    clock: RmiiClockConfig::InternalApll {
        gpio: ClkGpio::Gpio17,
        xtal: XtalFreq::Mhz40,
    },
    pins: RmiiPins { mdc: 23, mdio: 18 },
});

/// Shared state between the ISR and the embassy-net Driver.
static EMAC_STATE: EmacDriverState = EmacDriverState::new();

/// Hardware EMAC interrupt handler. Wakes `EMAC_STATE` so that
/// `EmacDriver` polls the next descriptor.
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

    // esp-rtos owns the embassy timer + scheduler. Required before any
    // `Timer::after(...)` or task spawning can fire. As of esp-rtos 0.3
    // the scheduler also needs the `FROM_CPU0` software interrupt to
    // drive context switches, so we hand it `software_interrupt0` from
    // `SoftwareInterruptControl`.
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    let mut delay = Delay::new();
    let rng = Rng::new();

    // SAFETY: `EMAC` is touched only here at init time — single owner.
    let emac = unsafe { &mut *core::ptr::addr_of_mut!(EMAC) };

    emac.set_mac_address(MAC_ADDRESS);
    emac.init(&mut delay).expect("EMAC init");
    emac.bind_interrupt(emac_interrupt_handler);

    let mut mdio = EspMdio::new();
    let mut phy = PhyLan87xx::new(PHY_ADDR);
    phy.init(&mut mdio).expect("PHY init");

    // Block until the PHY reports link up. `EmacDriver` only delivers
    // frames to embassy-net when `EMAC_STATE.link_up == true`, so DHCP
    // cannot complete before this point regardless of cable state. A
    // production firmware would spawn a background task to poll
    // `phy.poll_link` periodically and call
    // `EMAC_STATE.set_link_up() / set_link_down()` on transitions; this
    // example keeps things linear for clarity.
    let status = loop {
        match phy.poll_link(&mut mdio) {
            Ok(Some(s)) => break s,
            Ok(None) => {}
            Err(_) => {} // transient MDIO read errors at very early boot
        }
        delay.delay_millis(200);
    };

    // `Speed` / `Duplex` are re-exports of the trait-crate types
    // under the `mdio-phy` feature, so the PHY's `LinkStatus` lands
    // directly into `set_speed` / `set_duplex` with no conversion.
    emac.set_speed(status.speed);
    emac.set_duplex(status.duplex);
    EMAC_STATE.set_link_up();

    emac.start().expect("EMAC start");

    // `EmacDefaultDriver` is the type alias for the default ring sizing
    // — currently `EmacDriver<'_, 10, 10, 1600>`, with the const
    // generics sourced from `DEFAULT_RX` / `DEFAULT_TX` / `DEFAULT_BUF`.
    // Its inherent `new` is the same constructor, just without having
    // to spell out the const generics again at the call site.
    let driver = EmacDefaultDriver::new(emac, &EMAC_STATE);
    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);

    static RESOURCES: StaticCell<StackResources<8>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        driver,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        RESOURCES.init(StackResources::<8>::new()),
        net_seed,
    );

    // embassy-executor 0.10: `#[task]` functions return
    // `Result<SpawnToken<_>, SpawnError>`, and `Spawner::spawn` returns
    // `()`. So the fallible call is the task, not the spawn — match the
    // `expect("…")` style used by the rest of this example.
    spawner.spawn(net_task(runner).expect("net_task spawn"));

    // Hot loop: wait for DHCP, then idle. In a real app you'd hand the
    // stack to your TCP/UDP services here.
    wait_for_ip(stack).await;
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

async fn wait_for_ip(stack: Stack<'static>) {
    loop {
        if let Some(cfg) = stack.config_v4() {
            esp_println::println!("[net] got IP: {}", cfg.address);
            if let Some(gw) = cfg.gateway {
                esp_println::println!("[net] gateway: {}", gw);
            }
            for dns in cfg.dns_servers.as_slice() {
                esp_println::println!("[net] dns: {}", dns);
            }
            return;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}
