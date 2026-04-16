// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! ESP32 EMAC Ethernet MAC driver.
//!
//! Phase 1 of the esp-emac migration: delegates the MAC/DMA work to
//! [`ph_esp32_mac`] while exposing the esp-emac public surface that
//! firmware depends on (our [`EmacConfig`], [`Emac`], [`EspMdio`],
//! embassy-net integration). APLL 50 MHz clock generation and the
//! RMII clock-pin setup stay in our [`clock`] module — ph-esp32-mac
//! does not handle those.
//!
//! Future phases will replace the internal delegations piece by piece
//! (see `docs/plans/esp-emac-migration.md` in the firmware repo).

#![no_std]

pub mod clock;
pub mod config;
pub mod emac;
#[cfg(feature = "embassy-net")]
pub mod embassy;
pub mod error;
pub mod mdio;

// Legacy modules kept for reference during the phased rewrite. They are
// not wired into the current implementation (the facade delegates to
// ph-esp32-mac) and will be removed once the per-module rewrite lands.
#[doc(hidden)]
#[allow(clippy::assertions_on_constants)]
pub mod dma;
#[doc(hidden)]
#[allow(clippy::assertions_on_constants)]
pub mod regs;

pub use config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins};
pub use emac::{Duplex, Emac, EmacDefault, EmacSmall, EmacState, Speed};
pub use error::EmacError;
pub use mdio::{EspMdio, MdcClockDivider};
