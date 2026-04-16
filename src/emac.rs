// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Main EMAC driver struct (Phase 1 facade).
//!
//! This implementation delegates the EMAC MAC/DMA work to
//! [`ph_esp32_mac::Emac`] while keeping the esp-emac public surface the
//! firmware depends on. It also keeps the APLL 50 MHz clock and GPIO
//! clock-output setup in our [`crate::clock`] module, which ph-esp32-mac
//! does not handle.
//!
//! # Init Sequence
//!
//! 1. Configure APLL 50 MHz and clock GPIO (ours, for `InternalApll` mode)
//!    — or configure GPIO as clock input (`External` mode).
//! 2. Delegate to [`ph_esp32_mac::Emac::init`] which handles SMI pins,
//!    RMII data pins, DPORT clock, PHY interface regs, software reset,
//!    MAC/DMA defaults, descriptor chains, and MAC address programming.
//!
//! # Migration note
//!
//! The crate will replace these delegations piece by piece in future
//! phases (see `docs/plans/esp-emac-migration.md` in the firmware repo).

use embedded_hal::delay::DelayNs;

use crate::config::{ClkGpio, EmacConfig, RmiiClockConfig};
use crate::error::EmacError;

use ph_esp32_mac::{
    DmaBurstLen as PhDmaBurstLen, Duplex as PhDuplex, EmacConfig as PhEmacConfig,
    PhyInterface as PhPhyInterface, RmiiClockMode as PhRmiiClockMode, Speed as PhSpeed,
    State as PhState,
};

// =============================================================================
// Link parameter types (stable public surface)
// =============================================================================

/// Link speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// 10 Mbps.
    Mbps10,
    /// 100 Mbps.
    Mbps100,
}

/// Duplex mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Duplex {
    /// Half duplex.
    Half,
    /// Full duplex.
    Full,
}

#[cfg(feature = "mdio-phy")]
impl From<eth_mdio_phy::Speed> for Speed {
    fn from(s: eth_mdio_phy::Speed) -> Self {
        match s {
            eth_mdio_phy::Speed::Mbps10 => Speed::Mbps10,
            eth_mdio_phy::Speed::Mbps100 => Speed::Mbps100,
        }
    }
}

#[cfg(feature = "mdio-phy")]
impl From<eth_mdio_phy::Duplex> for Duplex {
    fn from(d: eth_mdio_phy::Duplex) -> Self {
        match d {
            eth_mdio_phy::Duplex::Half => Duplex::Half,
            eth_mdio_phy::Duplex::Full => Duplex::Full,
        }
    }
}

impl From<Speed> for PhSpeed {
    fn from(s: Speed) -> Self {
        match s {
            Speed::Mbps10 => PhSpeed::Mbps10,
            Speed::Mbps100 => PhSpeed::Mbps100,
        }
    }
}

impl From<Duplex> for PhDuplex {
    fn from(d: Duplex) -> Self {
        match d {
            Duplex::Half => PhDuplex::Half,
            Duplex::Full => PhDuplex::Full,
        }
    }
}

// =============================================================================
// Driver state
// =============================================================================

/// EMAC driver state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmacState {
    /// Not yet initialized.
    Uninitialized,
    /// Initialized but not running.
    Initialized,
    /// Running (DMA active, can transmit/receive).
    Running,
}

impl From<PhState> for EmacState {
    fn from(s: PhState) -> Self {
        match s {
            PhState::Uninitialized => EmacState::Uninitialized,
            PhState::Initialized | PhState::Stopped => EmacState::Initialized,
            PhState::Running => EmacState::Running,
        }
    }
}

// =============================================================================
// EMAC driver (facade)
// =============================================================================

/// ESP32 EMAC driver with static DMA buffers.
///
/// # Const Generics
/// - `RX`: number of RX descriptors (default 10)
/// - `TX`: number of TX descriptors (default 10)
/// - `BUF`: buffer size per descriptor in bytes (default 1600)
pub struct Emac<const RX: usize = 10, const TX: usize = 10, const BUF: usize = 1600> {
    inner: ph_esp32_mac::Emac<RX, TX, BUF>,
    config: EmacConfig,
    mac_address: [u8; 6],
}

impl<const RX: usize, const TX: usize, const BUF: usize> Emac<RX, TX, BUF> {
    /// Create a new EMAC driver (not yet initialized).
    pub const fn new(config: EmacConfig) -> Self {
        Self {
            inner: ph_esp32_mac::Emac::new(),
            config,
            mac_address: [0; 6],
        }
    }

    // ── State accessors ─────────────────────────────────────────────────

    /// Get current driver state.
    #[inline(always)]
    pub fn state(&self) -> EmacState {
        self.inner.state().into()
    }

    /// Get the configured MAC address.
    #[inline(always)]
    pub fn mac_address(&self) -> [u8; 6] {
        self.mac_address
    }

    /// Get a reference to the current configuration.
    #[inline(always)]
    pub fn config(&self) -> &EmacConfig {
        &self.config
    }

    /// Set MAC address.
    ///
    /// If the driver has been initialized, the hardware MAC address
    /// filter registers are updated immediately.
    pub fn set_mac_address(&mut self, mac: [u8; 6]) {
        self.mac_address = mac;
        if self.inner.state() != PhState::Uninitialized {
            self.inner.set_mac_address(&mac);
        }
    }

    /// Set link speed. Call after PHY reports link status change.
    pub fn set_speed(&mut self, speed: Speed) {
        if self.inner.state() == PhState::Uninitialized {
            return;
        }
        self.inner.set_speed(speed.into());
    }

    /// Set duplex mode. Call after PHY reports link status change.
    pub fn set_duplex(&mut self, duplex: Duplex) {
        if self.inner.state() == PhState::Uninitialized {
            return;
        }
        self.inner.set_duplex(duplex.into());
    }

    /// Total static memory used by this EMAC instance.
    pub const fn memory_usage() -> usize {
        ph_esp32_mac::Emac::<RX, TX, BUF>::memory_usage()
    }

    // ── Initialization ──────────────────────────────────────────────────

    /// Initialize the EMAC peripheral.
    ///
    /// Configures APLL + clock GPIO (our clock module), then delegates
    /// to [`ph_esp32_mac::Emac::init`] for SMI/RMII pins, clocks,
    /// software reset, MAC/DMA setup, and MAC address programming.
    ///
    /// Must be called with the [`Emac`] already in its final memory
    /// location (the DMA descriptor chain is self-referential).
    ///
    /// # Errors
    ///
    /// Returns [`EmacError`] if the underlying driver's init fails
    /// (e.g. [`EmacError::Timeout`] on DMA reset timeout).
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), EmacError> {
        // 1. Clock GPIO + APLL (or input pad for external clock).
        //    ph-esp32-mac does not handle APLL — we do it here.
        self.configure_clock();

        // 2. Build ph-esp32-mac config from ours + MAC address.
        let ph_config = self.build_ph_config();

        // 3. Delegate the MAC/DMA init.
        self.inner.init(ph_config, delay)?;

        Ok(())
    }

    /// Start the EMAC (enable DMA TX/RX and MAC TX/RX).
    ///
    /// After this call the driver is in [`EmacState::Running`] and can
    /// transmit and receive frames.
    pub fn enable(&mut self) {
        let _ = self.inner.start();
    }

    /// Stop the EMAC.
    pub fn disable(&mut self) {
        let _ = self.inner.stop();
    }

    // ── TX / RX ─────────────────────────────────────────────────────────

    /// Transmit a frame.
    ///
    /// Returns the number of bytes submitted.
    pub fn transmit(&mut self, data: &[u8]) -> Result<usize, EmacError> {
        Ok(self.inner.transmit(data)?)
    }

    /// Receive a frame.
    ///
    /// Returns `Ok(Some(len))` on a good frame, `Ok(None)` when no
    /// frame is available, or an error.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<Option<usize>, EmacError> {
        if !self.inner.rx_available() {
            return Ok(None);
        }
        match self.inner.receive(buffer) {
            Ok(len) => Ok(Some(len)),
            Err(ph_esp32_mac::Error::Io(ph_esp32_mac::IoError::IncompleteFrame)) => Ok(None),
            Err(e) => Err(e.into()),
        }
    }

    /// Check whether a received frame is available.
    #[inline(always)]
    pub fn rx_available(&self) -> bool {
        self.inner.rx_available()
    }

    /// Check whether the TX ring has room for a frame of `len` bytes.
    #[inline(always)]
    pub fn can_transmit(&self, len: usize) -> bool {
        self.inner.can_transmit(len)
    }

    // ── Interrupt helpers ───────────────────────────────────────────────

    /// Read DMA interrupt status register (raw bitfield).
    pub fn interrupt_status(&self) -> u32 {
        self.inner.interrupt_status().to_raw()
    }

    /// Clear DMA interrupt status flags (write-1-to-clear).
    pub fn clear_interrupts(&self, flags: u32) {
        let status = ph_esp32_mac::InterruptStatus::from_raw(flags);
        self.inner.clear_interrupts(status);
    }

    /// Enable default DMA interrupts (TX, RX complete).
    pub fn enable_interrupts(&self) {
        self.inner.enable_tx_interrupt(true);
        self.inner.enable_rx_interrupt(true);
    }

    // ── Phase 1 facade escape hatch ─────────────────────────────────────

    /// Access the underlying `ph-esp32-mac` driver.
    ///
    /// Used by the embassy-net integration to hand the inner driver to
    /// `ph_esp32_mac::integration::embassy_net::EmbassyEmac`. Removed when
    /// esp-emac becomes a standalone implementation (see migration plan).
    #[doc(hidden)]
    pub fn inner_mut(&mut self) -> &mut ph_esp32_mac::Emac<RX, TX, BUF> {
        &mut self.inner
    }

    // ── Internal helpers ────────────────────────────────────────────────

    fn configure_clock(&self) {
        match self.config.clock {
            RmiiClockConfig::InternalApll { gpio } => {
                crate::clock::configure_apll_50mhz();
                crate::clock::configure_emac_clk_out(gpio);
            }
            RmiiClockConfig::External { gpio } => {
                crate::clock::configure_emac_clk_in(gpio);
            }
        }
    }

    fn build_ph_config(&self) -> PhEmacConfig {
        let rmii_clock = match self.config.clock {
            RmiiClockConfig::InternalApll { gpio } => PhRmiiClockMode::InternalOutput {
                gpio: clk_gpio_to_u8(gpio),
            },
            RmiiClockConfig::External { gpio } => PhRmiiClockMode::ExternalInput {
                gpio: clk_gpio_to_u8(gpio),
            },
        };

        PhEmacConfig {
            phy_interface: PhPhyInterface::Rmii,
            rmii_clock,
            mac_address: self.mac_address,
            dma_burst_len: PhDmaBurstLen::Burst32,
            ..PhEmacConfig::new()
        }
    }
}

impl<const RX: usize, const TX: usize, const BUF: usize> Default for Emac<RX, TX, BUF> {
    fn default() -> Self {
        Self::new(EmacConfig {
            clock: RmiiClockConfig::InternalApll {
                gpio: ClkGpio::Gpio17,
            },
            pins: crate::config::RmiiPins::default(),
        })
    }
}

/// Convenience alias: 10 RX / 10 TX / 1600-byte buffers (32 KB SRAM).
pub type EmacDefault = Emac<10, 10, 1600>;

/// Convenience alias: 4 RX / 4 TX / 1600-byte buffers (13 KB SRAM).
pub type EmacSmall = Emac<4, 4, 1600>;

// =============================================================================
// Helpers
// =============================================================================

const fn clk_gpio_to_u8(gpio: ClkGpio) -> u8 {
    gpio.gpio_num()
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speed_roundtrip() {
        assert_eq!(PhSpeed::from(Speed::Mbps10), PhSpeed::Mbps10);
        assert_eq!(PhSpeed::from(Speed::Mbps100), PhSpeed::Mbps100);
    }

    #[test]
    fn duplex_roundtrip() {
        assert_eq!(PhDuplex::from(Duplex::Full), PhDuplex::Full);
        assert_eq!(PhDuplex::from(Duplex::Half), PhDuplex::Half);
    }

    #[test]
    fn state_maps_stopped_to_initialized() {
        assert_eq!(
            EmacState::from(PhState::Uninitialized),
            EmacState::Uninitialized
        );
        assert_eq!(
            EmacState::from(PhState::Initialized),
            EmacState::Initialized
        );
        assert_eq!(EmacState::from(PhState::Stopped), EmacState::Initialized);
        assert_eq!(EmacState::from(PhState::Running), EmacState::Running);
    }
}
