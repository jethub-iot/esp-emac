// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! ESP32 EMAC Ethernet MAC driver.
//!
//! Native MAC/DMA bring-up via [`crate::emac::Emac`]. Register helpers
//! (`MacRegs`, `DmaRegs`, `ExtRegs`, `GpioMatrix`, `ResetController`)
//! are still imported from [`ph_esp32_mac::unsafe_registers`] in this
//! phase and will be copied into our own `regs/*` modules in a follow-up
//! before `ph-esp32-mac` can be dropped as a dependency.

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

pub use config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins};
pub use emac::{Duplex, Emac, EmacDefault, EmacSmall, EmacState, Speed};
pub use error::EmacError;
pub use interrupt::InterruptStatus;
pub use mdio::{EspMdio, MdcClockDivider};
