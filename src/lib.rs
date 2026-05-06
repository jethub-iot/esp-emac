// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Native ESP32 Ethernet MAC driver for `#![no_std]` Rust.
//!
//! Owns the DMA engine and brings the EMAC peripheral up directly via
//! memory-mapped register helpers — no `ph-esp32-mac`, no `esp-idf-svc`,
//! no `esp-eth`.
//!
//! Pairs with [`eth-phy-lan87xx`](https://crates.io/crates/eth-phy-lan87xx)
//! (or any [`eth_mdio_phy::PhyDriver`](https://docs.rs/eth-mdio-phy)
//! implementation) for the PHY side, and with
//! [`embassy-net`](https://crates.io/crates/embassy-net) for the
//! TCP/IP stack.
//!
//! See the crate [README](https://github.com/jethub-iot/esp-emac-rs#readme)
//! for an installation guide, embassy-net example, ISR setup, and
//! troubleshooting checklist.
//!
//! # Quick start (embassy-net + LAN8720A)
//!
//! ```no_run
//! # #![cfg_attr(not(feature = "embassy-net"), allow(unused))]
//! # #[cfg(feature = "embassy-net")]
//! # mod __doc {
//! use esp_emac::config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins, XtalFreq};
//! use esp_emac::EmacDefault;
//! use esp_emac::embassy::EmacDriverState;
//!
//! // `Emac::new` (and therefore `EmacDefault::new`) is a `const fn`.
//! // Storing the EMAC in a `static mut` gives compile-time BSS init
//! // — no runtime stack temporary, deterministic on cold boot.
//! //
//! // The default ring sizing is currently 10 RX / 10 TX / 1600-byte
//! // buffers (~32 KiB), sourced from `DEFAULT_RX` / `DEFAULT_TX` /
//! // `DEFAULT_BUF`. At that size a `StaticCell::init(EmacDefault::new(..))`
//! // pattern would risk materialising the full struct on the caller's
//! // stack before moving it into static storage; `static mut` avoids
//! // that path entirely.
//! static mut EMAC: EmacDefault = EmacDefault::new(EmacConfig {
//!     clock: RmiiClockConfig::InternalApll {
//!         gpio: ClkGpio::Gpio17,
//!         xtal: XtalFreq::Mhz40,
//!     },
//!     pins: RmiiPins { mdc: 23, mdio: 18 },
//! });
//! static EMAC_STATE: EmacDriverState = EmacDriverState::new();
//!
//! // In `main`, take the `&'static mut` once. SAFETY: `EMAC` is touched
//! // only here — single owner — so there is no aliasing.
//! # fn doc() {
//! let _emac: &'static mut EmacDefault =
//!     unsafe { &mut *core::ptr::addr_of_mut!(EMAC) };
//! # }
//! # }
//! ```
//!
//! Full bring-up (PHY init, link wait, embassy-net plumbing, DHCP) is
//! shown in [`examples/embassy_net_lan8720a.rs`](https://github.com/jethub-iot/esp-emac-rs/blob/main/examples/embassy_net_lan8720a.rs).
//!
//! # Crate features
//!
//! | Feature | Default | When to enable |
//! | --- | --- | --- |
//! | `esp-hal` | off | Always for hardware bring-up — pulls in [`esp_hal::interrupt`] for ISR binding. |
//! | `mdio-phy` | off | When using a `PhyDriver`-based PHY driver via [`mdio::EspMdio`]. |
//! | `embassy-net` | off | When using [`embassy-net`] — exposes [`embassy::EmacDriver`]. |
//! | `async` | off | When using [`reset::async_impl::AsyncResetController`]. |
//! | `defmt` | off | Adds `defmt::Format` derives on public types. |
//!
//! # Compatibility
//!
//! - **Target:** `xtensa-esp32-none-elf` (original ESP32, Xtensa LX6)
//! - **MSRV:** 1.88 (constrained by `esp-hal = "1.1"`'s declared `rust-version`)
//! - **`esp-hal`:** 1.1.x
//! - **`embassy-net`:** 0.9.x
//! - **`embassy-executor`:** 0.10.x
//!
//! Other ESP variants (S2/S3/C-series/H2) have **no** built-in EMAC.
//! ESP32-P4 has a newer Synopsys GMAC revision and is not yet supported
//! (planned through a chip-feature split in `regs/*`).
//!
//! Pure register-arithmetic unit tests build and run on the host
//! (`cargo test --target $HOST_TARGET`), which is how `regs/*` is
//! exercised in CI.

#![no_std]

pub mod clock;
pub mod config;
pub mod dma;
pub mod emac;
#[cfg(feature = "embassy-net")]
pub mod embassy;
pub mod error;
pub mod interrupt;
pub mod mdio;
pub mod regs;
pub mod reset;

pub use config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins, XtalFreq};
#[cfg(feature = "mdio-phy")]
pub use emac::{Duplex, Speed};
pub use emac::{Emac, EmacDefault, EmacSmall, EmacState};
pub use error::EmacError;
pub use interrupt::InterruptStatus;
pub use mdio::{EspMdio, MdcClockDivider};
