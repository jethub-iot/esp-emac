// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! ESP32 EMAC bare-metal Ethernet MAC driver.
//!
//! Provides DMA ring management, RMII interface configuration,
//! APLL 50 MHz clock generation, and MDIO bus controller.
//!
//! This crate is ESP32-specific and designed to be eventually
//! replaced by native esp-hal Ethernet support.

#![no_std]

pub mod config;
pub mod dma;
pub mod error;
pub mod mdio;
pub mod regs;

pub use config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins};
pub use error::EmacError;
pub use mdio::{EspMdio, MdcClockDivider};
