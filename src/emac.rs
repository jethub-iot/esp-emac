// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Native ESP32 EMAC driver — Phase 3 implementation.
//!
//! Owns the DMA engine, drives the bring-up sequence directly via register
//! helpers, and no longer wraps `ph_esp32_mac::Emac`. This is the staging
//! point of the migration: register helpers (`MacRegs`, `DmaRegs`,
//! `ExtRegs`, `GpioMatrix`, `ResetController`) are still imported from
//! `ph_esp32_mac::unsafe_registers` while we copy them into our own
//! `regs/*` modules in a follow-up.

use embedded_hal::delay::DelayNs;

use ph_esp32_mac::unsafe_registers::{DmaRegs, ExtRegs, GpioMatrix, ResetController};

use crate::regs::mac as mac_regs;

use crate::config::{ClkGpio, EmacConfig, RmiiClockConfig};
use crate::dma::engine::DmaEngine;
use crate::error::EmacError;
use crate::interrupt::InterruptStatus;
use crate::regs::dma::{bus_mode, operation};
use crate::regs::mac::{config, frame_filter};

const SOFT_RESET_TIMEOUT_MS: u32 = 100;
const TX_FIFO_FLUSH_TIMEOUT_US: u32 = 100_000;

// =============================================================================
// Link parameters and driver state
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

/// EMAC driver state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmacState {
    /// Not yet initialized.
    Uninitialized,
    /// `init()` succeeded but DMA/MAC are not running.
    Initialized,
    /// `start()` succeeded — DMA active, can transmit/receive.
    Running,
}

// =============================================================================
// EMAC driver
// =============================================================================

/// ESP32 EMAC driver with statically allocated DMA buffers.
///
/// The DMA descriptor chain is self-referential, so the driver MUST be
/// placed in its final memory location BEFORE [`init`](Self::init) is
/// called.
pub struct Emac<const RX: usize = 10, const TX: usize = 10, const BUF: usize = 1600> {
    dma: DmaEngine<RX, TX, BUF>,
    config: EmacConfig,
    state: EmacState,
    mac_address: [u8; 6],
}

impl<const RX: usize, const TX: usize, const BUF: usize> Emac<RX, TX, BUF> {
    /// Create a new (uninitialized) driver.
    pub const fn new(config: EmacConfig) -> Self {
        Self {
            dma: DmaEngine::new(),
            config,
            state: EmacState::Uninitialized,
            mac_address: [0; 6],
        }
    }

    // ── State accessors ────────────────────────────────────────────────────

    #[inline(always)]
    pub fn state(&self) -> EmacState {
        self.state
    }

    #[inline(always)]
    pub fn mac_address(&self) -> [u8; 6] {
        self.mac_address
    }

    #[inline(always)]
    pub fn config(&self) -> &EmacConfig {
        &self.config
    }

    /// Total static memory used by this EMAC instance.
    pub const fn memory_usage() -> usize {
        DmaEngine::<RX, TX, BUF>::memory_usage()
    }

    // ── Configuration ──────────────────────────────────────────────────────

    /// Set the MAC address.
    ///
    /// If the driver has been initialized, the hardware filter registers
    /// are updated immediately.
    pub fn set_mac_address(&mut self, mac: [u8; 6]) {
        self.mac_address = mac;
        if self.state != EmacState::Uninitialized {
            crate::regs::mac::set_mac_address(&mac);
        }
    }

    /// Apply the link speed reported by the PHY.
    pub fn set_speed(&mut self, speed: Speed) {
        if self.state == EmacState::Uninitialized {
            return;
        }
        mac_regs::set_speed_100mbps(matches!(speed, Speed::Mbps100));
    }

    /// Apply the duplex mode reported by the PHY.
    pub fn set_duplex(&mut self, duplex: Duplex) {
        if self.state == EmacState::Uninitialized {
            return;
        }
        mac_regs::set_duplex_full(matches!(duplex, Duplex::Full));
    }

    // ── Initialization ─────────────────────────────────────────────────────

    /// Initialize the EMAC peripheral.
    ///
    /// Sequence (mirrors the canonical ESP32 GMAC bring-up):
    /// 1. APLL 50 MHz + RMII clock GPIO.
    /// 2. SMI + RMII pin routing.
    /// 3. DPORT EMAC peripheral clock enable.
    /// 4. PHY interface mode (RMII) + clock source select.
    /// 5. EMAC extension clocks + RAM power-up.
    /// 6. DMA software reset.
    /// 7. MAC config defaults (PS/FES/DM/ACS/JD/WD).
    /// 8. DMA bus mode + operation mode defaults.
    /// 9. DMA descriptor chains and base-address registers.
    /// 10. MAC address program.
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), EmacError> {
        if self.state != EmacState::Uninitialized {
            return Err(EmacError::AlreadyInitialized);
        }

        // 1. Clock GPIO + APLL (or input pad for external clock).
        self.configure_clock();

        // 2. Configure SMI pins (MDC=GPIO23, MDIO=GPIO18) and RMII data
        //    pins (fixed function 5).
        if matches!(self.config.clock, RmiiClockConfig::External { .. }) {
            ExtRegs::configure_gpio0_rmii_clock_input();
        }
        GpioMatrix::configure_smi_pins();
        GpioMatrix::configure_rmii_pins();

        // 3. Enable EMAC peripheral clock through DPORT.
        ExtRegs::enable_peripheral_clock();

        // 4. PHY interface — RMII with the appropriate clock source.
        ExtRegs::set_rmii_mode();
        match self.config.clock {
            RmiiClockConfig::External { .. } => ExtRegs::set_rmii_clock_external(),
            RmiiClockConfig::InternalApll { .. } => ExtRegs::set_rmii_clock_internal(),
        }

        // 5. EMAC extension clocks + RAM power.
        ExtRegs::enable_clocks();
        ExtRegs::power_up_ram();

        // 6. Software reset of the DMA controller.
        let mut reset_ctrl =
            ResetController::with_timeout(BorrowedDelay(delay), SOFT_RESET_TIMEOUT_MS);
        reset_ctrl.soft_reset().map_err(|_| EmacError::Timeout)?;

        // 7. MAC configuration defaults: 100 Mbps full duplex, port select,
        //    auto pad/CRC strip, jabber + watchdog disabled.
        let mac_cfg = config::PORT_SELECT
            | config::SPEED_100
            | config::DUPLEX_FULL
            | config::AUTO_PAD_CRC_STRIP
            | config::JABBER_DISABLE
            | config::WATCHDOG_DISABLE;
        mac_regs::set_config(mac_cfg);

        // Frame filter: pass all multicast (broadcast accepted by default).
        mac_regs::set_frame_filter(frame_filter::PASS_ALL_MULTICAST);
        mac_regs::set_hash_table(0);

        // 8. DMA bus mode and operation mode.
        //
        // ATDS = enhanced 8-word descriptor layout (32 bytes per
        // descriptor). Our `dma::descriptor::{TxDescriptor,
        // RxDescriptor}` are now 8 words to match.
        let pbl = 32u32;
        let bus = bus_mode::FIXED_BURST
            | bus_mode::AAL
            | bus_mode::USP
            | bus_mode::ATDS
            | ((pbl << bus_mode::PBL_SHIFT) & bus_mode::PBL_MASK);
        DmaRegs::set_bus_mode(bus);
        DmaRegs::set_operation_mode(operation::TSF | operation::RSF);
        DmaRegs::disable_all_interrupts();
        DmaRegs::clear_all_interrupts();

        // 9. Descriptor chains. Returns physical base addresses suitable for
        //    DMARXBASEADDR / DMATXBASEADDR.
        let (rx_base, tx_base) = self.dma.init();
        DmaRegs::set_rx_desc_list_addr(rx_base);
        DmaRegs::set_tx_desc_list_addr(tx_base);

        // 10. Programme the MAC address into ADDR0H / ADDR0L (with AE bit).
        // The internal filter latch on this Synopsys GMAC fires on the LOW
        // write — `regs::mac::set_mac_address` writes HIGH first to keep
        // the AE bit, then LOW to trigger the latch.
        crate::regs::mac::set_mac_address(&self.mac_address);

        self.state = EmacState::Initialized;
        Ok(())
    }

    /// Start TX/RX (DMA + MAC).
    pub fn start(&mut self) -> Result<(), EmacError> {
        match self.state {
            EmacState::Initialized => {}
            EmacState::Running => return Ok(()),
            EmacState::Uninitialized => return Err(EmacError::NotInitialized),
        }

        // Reset descriptor ownership in case of a previous run.
        let (_rx_base, _tx_base) = self.dma.reset();

        DmaRegs::clear_all_interrupts();
        DmaRegs::enable_default_interrupts();

        // Enable MAC TX, then DMA TX, DMA RX, then MAC RX (matches the
        // ordering from the ESP32 reference manual / IDF EMAC driver).
        let cfg = mac_regs::config();
        mac_regs::set_config(cfg | config::TX_ENABLE);

        DmaRegs::start_tx();
        DmaRegs::start_rx();

        let cfg = mac_regs::config();
        mac_regs::set_config(cfg | config::RX_ENABLE);

        // Issue an RX poll demand so the DMA does not stay in Suspended
        // state if all descriptors were already CPU-owned.
        DmaRegs::rx_poll_demand();

        self.state = EmacState::Running;
        Ok(())
    }

    /// Stop TX/RX.
    pub fn stop(&mut self) -> Result<(), EmacError> {
        if self.state != EmacState::Running {
            return Err(EmacError::NotInitialized);
        }

        // Stop DMA TX, wait for in-flight data to drain (best effort).
        DmaRegs::stop_tx();

        // Flush TX FIFO and wait for the bit to self-clear.
        DmaRegs::flush_tx_fifo();
        let mut waited_us = 0u32;
        while waited_us < TX_FIFO_FLUSH_TIMEOUT_US {
            if (DmaRegs::operation_mode() & operation::FTF) == 0 {
                break;
            }
            waited_us += 10;
        }

        // Disable MAC TX and RX, then DMA RX.
        let cfg = mac_regs::config();
        mac_regs::set_config(cfg & !(config::TX_ENABLE | config::RX_ENABLE));

        DmaRegs::stop_rx();
        DmaRegs::disable_all_interrupts();

        self.state = EmacState::Initialized;
        Ok(())
    }

    // ── Frame I/O ─────────────────────────────────────────────────────────

    /// Transmit a frame (blocking on descriptor availability is not
    /// performed — caller must check [`can_transmit`](Self::can_transmit)
    /// or be ready to receive `EmacError::TxBufferFull`).
    pub fn transmit(&mut self, data: &[u8]) -> Result<usize, EmacError> {
        if self.state != EmacState::Running {
            return Err(EmacError::NotInitialized);
        }
        let n = self.dma.transmit(data)?;
        // Kick TX DMA out of suspended state if we just refilled descriptors.
        DmaRegs::tx_poll_demand();
        Ok(n)
    }

    /// Receive a frame, if any.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<Option<usize>, EmacError> {
        if self.state != EmacState::Running {
            return Err(EmacError::NotInitialized);
        }
        let result = self.dma.receive(buffer)?;
        if result.is_some() {
            // After freeing a descriptor, kick the RX poll demand so the
            // DMA leaves Suspended state.
            DmaRegs::rx_poll_demand();
        }
        Ok(result)
    }

    /// Whether a received frame is currently waiting in the ring.
    #[inline(always)]
    pub fn rx_available(&self) -> bool {
        self.dma.rx_available()
    }

    /// Whether the TX ring has room for a frame of `len` bytes.
    #[inline(always)]
    pub fn can_transmit(&self, len: usize) -> bool {
        self.dma.can_transmit(len)
    }

    /// Whether at least one TX descriptor is available for the next frame.
    #[inline(always)]
    pub fn tx_ready(&self) -> bool {
        self.dma.tx_available() > 0
    }

    // ── Interrupt helpers ──────────────────────────────────────────────────

    /// Bind an interrupt handler to the EMAC peripheral and enable the
    /// interrupt at the chip level.
    #[cfg(feature = "esp-hal")]
    pub fn bind_interrupt(&mut self, handler: esp_hal::interrupt::InterruptHandler) {
        use esp_hal::peripherals::Interrupt;

        for core in esp_hal::system::Cpu::other() {
            esp_hal::interrupt::disable(core, Interrupt::ETH_MAC);
        }
        esp_hal::interrupt::bind_handler(Interrupt::ETH_MAC, handler);
        let _ = esp_hal::interrupt::enable(Interrupt::ETH_MAC, handler.priority());
    }

    /// Disable the EMAC interrupt at the chip level.
    #[cfg(feature = "esp-hal")]
    pub fn disable_interrupt(&mut self) {
        use esp_hal::peripherals::Interrupt;
        esp_hal::interrupt::disable(esp_hal::system::Cpu::current(), Interrupt::ETH_MAC);
    }

    /// Read and parse the DMA status register.
    pub fn interrupt_status(&self) -> InterruptStatus {
        // SAFETY: read from a known-valid memory-mapped register.
        let raw = unsafe {
            core::ptr::read_volatile(
                (crate::regs::dma::BASE + crate::regs::dma::DMASTATUS) as *const u32,
            )
        };
        InterruptStatus::from_raw(raw)
    }

    /// Clear (write-1-to-clear) the given DMA status flags.
    pub fn clear_interrupts(&self, status: InterruptStatus) {
        let raw = status.to_raw();
        // SAFETY: write to a known-valid memory-mapped register.
        unsafe {
            core::ptr::write_volatile(
                (crate::regs::dma::BASE + crate::regs::dma::DMASTATUS) as *mut u32,
                raw,
            );
        }
    }

    /// Convenience: handle the ISR — read status, return parsed copy,
    /// clear flags. Equivalent to:
    /// ```ignore
    /// let s = emac.interrupt_status();
    /// emac.clear_interrupts(s);
    /// s
    /// ```
    pub fn handle_interrupt(&self) -> InterruptStatus {
        let s = self.interrupt_status();
        self.clear_interrupts(s);
        s
    }

    // ── Internal helpers ──────────────────────────────────────────────────

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

/// Convenience alias: 10 RX / 10 TX / 1600-byte buffers.
pub type EmacDefault = Emac<10, 10, 1600>;

/// Convenience alias: 4 RX / 4 TX / 1600-byte buffers.
pub type EmacSmall = Emac<4, 4, 1600>;

// =============================================================================
// Helpers
// =============================================================================

/// Wraps a `&mut DelayNs` so it can be passed by value to APIs that take
/// an owned `DelayNs` implementor (such as
/// `ph_esp32_mac::ResetController::with_timeout`).
struct BorrowedDelay<'a, D: DelayNs + ?Sized>(&'a mut D);

impl<D: DelayNs + ?Sized> DelayNs for BorrowedDelay<'_, D> {
    fn delay_ns(&mut self, ns: u32) {
        self.0.delay_ns(ns);
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_is_uninitialized() {
        let emac = EmacDefault::default();
        assert_eq!(emac.state(), EmacState::Uninitialized);
        assert_eq!(emac.mac_address(), [0u8; 6]);
    }

    #[test]
    fn set_mac_before_init_only_caches() {
        let mut emac = EmacDefault::default();
        let mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        emac.set_mac_address(mac);
        assert_eq!(emac.mac_address(), mac);
        // No register writes performed because state is Uninitialized.
    }

    #[test]
    fn memory_usage_matches_dma() {
        assert_eq!(
            EmacDefault::memory_usage(),
            DmaEngine::<10, 10, 1600>::memory_usage()
        );
    }
}
