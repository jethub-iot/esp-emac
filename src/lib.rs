// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! ESP32 EMAC Ethernet MAC driver.
//!
//! Native MAC/DMA bring-up via [`crate::emac::Emac`] using the local
//! `regs/*` modules and [`crate::reset::ResetController`]. No
//! `ph-esp32-mac` dependency at runtime.

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
pub use emac::{Duplex, Emac, EmacDefault, EmacSmall, EmacState, Speed};
pub use error::EmacError;
pub use interrupt::InterruptStatus;
pub use mdio::{EspMdio, MdcClockDivider};
