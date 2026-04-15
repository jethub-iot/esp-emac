// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Main EMAC driver struct.
//!
//! Ties together DMA engine, MAC/DMA/EXT registers, and configuration
//! into a single entry point for Ethernet MAC operation.
//!
//! # Init Sequence
//!
//! 1. Enable DPORT peripheral clock
//! 2. Configure SMI pins (MDC/MDIO via GPIO Matrix)
//! 3. Configure RMII data pins (IO_MUX function 5)
//! 4. Configure PHY interface (RMII mode, clock source)
//! 5. Enable extension clocks and power up RAM
//! 6. Software reset DMA
//! 7. Configure MAC defaults
//! 8. Configure DMA defaults
//! 9. Initialize DMA descriptor chains
//! 10. Program DMA base addresses
//! 11. Program MAC address
//!
//! # Important: Placement Before Init
//!
//! The DMA engine builds a self-referential descriptor chain.
//! **Place the `Emac` in its final memory location (e.g. `static`)
//! before calling [`Emac::init`].** Moving the struct after init
//! invalidates internal pointers and breaks DMA.

use crate::config::{ClkGpio, EmacConfig, RmiiClockConfig};
use crate::dma::engine::DmaEngine;
use crate::error::EmacError;

// =============================================================================
// Link parameter types (locally defined, no external dependency)
// =============================================================================

/// Link speed.
///
/// Mirrors `eth_mdio_phy::Speed` when the `mdio-phy` feature is active.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// 10 Mbps.
    Mbps10,
    /// 100 Mbps.
    Mbps100,
}

/// Duplex mode.
///
/// Mirrors `eth_mdio_phy::Duplex` when the `mdio-phy` feature is active.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Duplex {
    /// Half duplex.
    Half,
    /// Full duplex.
    Full,
}

// Feature-gated From conversions for eth-mdio-phy interop.
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

// =============================================================================
// Driver state
// =============================================================================

/// EMAC driver state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmacState {
    /// Not yet initialized.
    Uninitialized,
    /// Initialized but not running (DMA stopped).
    Initialized,
    /// Running (DMA active, can transmit/receive).
    Running,
}

// =============================================================================
// GPIO Matrix constants (ESP32)
// =============================================================================

/// GPIO peripheral base address.
const GPIO_BASE: usize = 0x3FF4_4000;

/// GPIO output enable set register (W1TS).
const GPIO_ENABLE_W1TS_OFFSET: usize = 0x24;

/// GPIO input function configuration register base offset.
/// For signal S: `GPIO_BASE + 0x130 + S*4`.
const GPIO_FUNC_IN_SEL_CFG_BASE: usize = 0x130;

/// GPIO output function configuration register base offset.
/// For GPIO N: `GPIO_BASE + 0x530 + N*4`.
const GPIO_FUNC_OUT_SEL_CFG_BASE: usize = 0x530;

/// EMAC MDC output signal index.
const EMAC_MDC_O_IDX: u32 = 200;
/// EMAC MDIO input signal index.
const EMAC_MDI_I_IDX: u32 = 201;
/// EMAC MDIO output signal index.
const EMAC_MDO_O_IDX: u32 = 201;

/// OEN_SEL bit — peripheral controls output enable.
const GPIO_OEN_SEL: u32 = 1 << 10;
/// SIG_IN_SEL bit — route through GPIO Matrix.
const GPIO_SIG_IN_SEL: u32 = 1 << 7;
/// GPIO_FUNC_IN_SEL mask (bits 5:0).
const GPIO_FUNC_IN_SEL_MASK: u32 = 0x3F;
/// GPIO_FUNC_OUT_SEL mask (bits 8:0).
const GPIO_FUNC_OUT_SEL_MASK: u32 = 0x1FF;

/// IO_MUX base address.
const IO_MUX_BASE: usize = 0x3FF4_9000;
/// IO_MUX MCU_SEL field mask (bits 14:12).
const IO_MUX_MCU_SEL_MASK: u32 = 0x07 << 12;
/// IO_MUX MCU_SEL shift.
const IO_MUX_MCU_SEL_SHIFT: u32 = 12;
/// IO_MUX FUN_IE (input enable) — bit 9.
const IO_MUX_FUN_IE: u32 = 1 << 9;
/// IO_MUX function value for GPIO Matrix routing.
const IO_MUX_FUNC_GPIO: u32 = 2;
/// EMAC IO_MUX function (function 5 for data pins and clock).
const IO_MUX_FUNC_EMAC: u32 = 5;
/// FUN_DRV mask (bits 11:10).
const IO_MUX_FUN_DRV_MASK: u32 = 0x03 << 10;

/// DMA software reset timeout (polling iterations).
const DMA_RESET_TIMEOUT: u32 = 10_000;

// =============================================================================
// EMAC driver
// =============================================================================

/// ESP32 EMAC driver with static DMA buffers.
///
/// Manages the EMAC peripheral: initialization, DMA, frame
/// transmission/reception, and link configuration.
///
/// # Const Generics
/// - `RX`: number of RX descriptors (default 10)
/// - `TX`: number of TX descriptors (default 10)
/// - `BUF`: buffer size per descriptor in bytes (default 1600)
pub struct Emac<const RX: usize = 10, const TX: usize = 10, const BUF: usize = 1600> {
    dma: DmaEngine<RX, TX, BUF>,
    config: EmacConfig,
    mac_address: [u8; 6],
    state: EmacState,
}

impl<const RX: usize, const TX: usize, const BUF: usize> Emac<RX, TX, BUF> {
    /// Create a new EMAC driver (not yet initialized).
    pub const fn new(config: EmacConfig) -> Self {
        Self {
            dma: DmaEngine::new(),
            config,
            mac_address: [0; 6],
            state: EmacState::Uninitialized,
        }
    }

    // ── State accessors ─────────────────────────────────────────────────

    /// Get current driver state.
    #[inline(always)]
    pub fn state(&self) -> EmacState {
        self.state
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
        if self.state != EmacState::Uninitialized {
            self.program_mac_address();
        }
    }

    /// Set link speed. Call after PHY reports link status change.
    pub fn set_speed(&mut self, speed: Speed) {
        if self.state == EmacState::Uninitialized {
            return;
        }
        // SAFETY: peripheral clock is enabled (state != Uninitialized).
        unsafe {
            match speed {
                Speed::Mbps100 => crate::regs::mac::set_bits(
                    crate::regs::mac::GMACCONFIG,
                    crate::regs::mac::config::SPEED_100,
                ),
                Speed::Mbps10 => crate::regs::mac::clear_bits(
                    crate::regs::mac::GMACCONFIG,
                    crate::regs::mac::config::SPEED_100,
                ),
            }
        }
    }

    /// Set duplex mode. Call after PHY reports link status change.
    pub fn set_duplex(&mut self, duplex: Duplex) {
        if self.state == EmacState::Uninitialized {
            return;
        }
        // SAFETY: peripheral clock is enabled (state != Uninitialized).
        unsafe {
            match duplex {
                Duplex::Full => crate::regs::mac::set_bits(
                    crate::regs::mac::GMACCONFIG,
                    crate::regs::mac::config::DUPLEX_FULL,
                ),
                Duplex::Half => crate::regs::mac::clear_bits(
                    crate::regs::mac::GMACCONFIG,
                    crate::regs::mac::config::DUPLEX_FULL,
                ),
            }
        }
    }

    /// Total static memory used by this EMAC instance (DMA buffers + descriptors).
    pub const fn memory_usage() -> usize {
        DmaEngine::<RX, TX, BUF>::memory_usage()
    }

    // ── Initialization ──────────────────────────────────────────────────

    /// Initialize the EMAC peripheral.
    ///
    /// Configures GPIO, clocks, resets DMA, sets up descriptors and MAC
    /// defaults. After init the driver is in [`EmacState::Initialized`].
    ///
    /// # Errors
    ///
    /// Returns [`EmacError::Timeout`] if the DMA software reset does
    /// not complete within the expected number of polling iterations.
    pub fn init(&mut self) -> Result<(), EmacError> {
        // 1. Enable DPORT peripheral clock.
        self.enable_peripheral_clock();

        // 2. Configure SMI pins (MDC/MDIO via GPIO Matrix).
        self.configure_smi_pins();

        // 3. Configure RMII data pins (fixed IO_MUX function 5).
        self.configure_rmii_data_pins();

        // 4. Configure PHY interface (RMII mode + clock source).
        self.configure_phy_interface();

        // 5. Enable extension clocks and power up RAM.
        self.enable_ext_clocks();

        // 6. DMA software reset.
        self.dma_software_reset()?;

        // 7. Configure MAC defaults.
        self.configure_mac_defaults();

        // 8. Configure DMA defaults (bus mode + operation mode).
        self.configure_dma_defaults();

        // 9. Initialize DMA engine (descriptor chains).
        let (rx_base, tx_base) = self.dma.init();

        // 10. Program DMA descriptor base addresses.
        self.program_dma_addresses(rx_base, tx_base);

        // 11. Program MAC address into filter registers.
        self.program_mac_address();

        self.state = EmacState::Initialized;
        Ok(())
    }

    /// Start the EMAC (enable DMA TX/RX, enable MAC TX/RX).
    ///
    /// After calling this the driver enters [`EmacState::Running`] and
    /// can transmit and receive frames.
    pub fn enable(&mut self) {
        if self.state == EmacState::Uninitialized {
            return;
        }

        // SAFETY: peripheral clock is enabled (state != Uninitialized).
        unsafe {
            // Clear pending DMA interrupts.
            crate::regs::dma::write(
                crate::regs::dma::DMASTATUS,
                crate::regs::dma::status::ALL_INTERRUPTS,
            );

            // Enable default DMA interrupts.
            crate::regs::dma::write(
                crate::regs::dma::DMAINTENABLE,
                crate::regs::dma::int_enable::DEFAULT,
            );

            // Enable MAC TX, then start DMA TX.
            crate::regs::mac::set_bits(
                crate::regs::mac::GMACCONFIG,
                crate::regs::mac::config::TX_ENABLE,
            );
            crate::regs::dma::set_bits(
                crate::regs::dma::DMAOPERATION,
                crate::regs::dma::operation::ST,
            );

            // Start DMA RX, then enable MAC RX.
            crate::regs::dma::set_bits(
                crate::regs::dma::DMAOPERATION,
                crate::regs::dma::operation::SR,
            );
            crate::regs::mac::set_bits(
                crate::regs::mac::GMACCONFIG,
                crate::regs::mac::config::RX_ENABLE,
            );

            // Issue RX poll demand so DMA starts fetching descriptors.
            crate::regs::dma::write(crate::regs::dma::DMARXPOLLDEMAND, 1);
        }

        self.state = EmacState::Running;
    }

    /// Stop the EMAC (disable DMA and MAC TX/RX).
    ///
    /// The driver returns to [`EmacState::Initialized`].
    pub fn disable(&mut self) {
        if self.state != EmacState::Running {
            return;
        }

        // SAFETY: peripheral clock is enabled (state == Running).
        unsafe {
            // Stop DMA TX.
            crate::regs::dma::clear_bits(
                crate::regs::dma::DMAOPERATION,
                crate::regs::dma::operation::ST,
            );

            // Stop DMA RX.
            crate::regs::dma::clear_bits(
                crate::regs::dma::DMAOPERATION,
                crate::regs::dma::operation::SR,
            );

            // Disable MAC TX/RX.
            crate::regs::mac::clear_bits(
                crate::regs::mac::GMACCONFIG,
                crate::regs::mac::config::TX_ENABLE | crate::regs::mac::config::RX_ENABLE,
            );

            // Disable DMA interrupts.
            crate::regs::dma::write(crate::regs::dma::DMAINTENABLE, 0);

            // Clear pending interrupts.
            crate::regs::dma::write(
                crate::regs::dma::DMASTATUS,
                crate::regs::dma::status::ALL_INTERRUPTS,
            );
        }

        self.state = EmacState::Initialized;
    }

    // ── TX / RX ─────────────────────────────────────────────────────────

    /// Transmit a frame.
    ///
    /// Copies `data` into the TX descriptor ring and triggers DMA TX
    /// poll demand. Returns the number of bytes submitted.
    ///
    /// # Errors
    ///
    /// - [`EmacError::NotInitialized`] if the driver is not running.
    /// - [`EmacError::InvalidLength`] if `data` is empty.
    /// - [`EmacError::FrameTooLarge`] if the frame exceeds ring capacity.
    /// - [`EmacError::NoDescriptorsAvailable`] if the ring is full.
    pub fn transmit(&mut self, data: &[u8]) -> Result<usize, EmacError> {
        if self.state != EmacState::Running {
            return Err(EmacError::NotInitialized);
        }
        let sent = self.dma.transmit(data)?;
        // SAFETY: peripheral clock is enabled.
        unsafe {
            crate::regs::dma::write(crate::regs::dma::DMATXPOLLDEMAND, 1);
        }
        Ok(sent)
    }

    /// Receive a frame.
    ///
    /// Copies the next received frame into `buffer`. Returns
    /// `Ok(Some(len))` with the frame length (excluding CRC),
    /// `Ok(None)` when no frame is available, or an error.
    ///
    /// # Errors
    ///
    /// - [`EmacError::NotInitialized`] if the driver is not running.
    /// - [`EmacError::BufferTooSmall`] if `buffer` is too small.
    /// - [`EmacError::FrameError`] if the received frame has errors.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<Option<usize>, EmacError> {
        if self.state != EmacState::Running {
            return Err(EmacError::NotInitialized);
        }
        self.dma.receive(buffer)
    }

    /// Check whether a received frame is available.
    #[inline(always)]
    pub fn rx_available(&self) -> bool {
        self.dma.rx_available()
    }

    /// Check whether the TX ring has room for a frame of `len` bytes.
    #[inline(always)]
    pub fn can_transmit(&self, len: usize) -> bool {
        self.dma.can_transmit(len)
    }

    // ── Interrupt helpers ───────────────────────────────────────────────

    /// Read DMA interrupt status register.
    pub fn interrupt_status(&self) -> u32 {
        // SAFETY: peripheral clock is enabled.
        unsafe { crate::regs::dma::read(crate::regs::dma::DMASTATUS) }
    }

    /// Clear DMA interrupt status flags (write-1-to-clear).
    pub fn clear_interrupts(&self, flags: u32) {
        // SAFETY: peripheral clock is enabled.
        unsafe {
            crate::regs::dma::write(crate::regs::dma::DMASTATUS, flags);
        }
    }

    /// Enable default DMA interrupts (TX, RX, fatal bus error, summaries).
    pub fn enable_interrupts(&self) {
        // SAFETY: peripheral clock is enabled.
        unsafe {
            crate::regs::dma::write(
                crate::regs::dma::DMAINTENABLE,
                crate::regs::dma::int_enable::DEFAULT,
            );
        }
    }

    // ── Private: hardware configuration ─────────────────────────────────

    /// Enable EMAC peripheral clock via DPORT_WIFI_CLK_EN_REG.
    fn enable_peripheral_clock(&self) {
        // SAFETY: DPORT register is always accessible.
        unsafe {
            let addr = crate::regs::ext::DPORT_WIFI_CLK_EN_REG;
            let val = core::ptr::read_volatile(addr as *const u32);
            core::ptr::write_volatile(
                addr as *mut u32,
                val | crate::regs::ext::DPORT_WIFI_CLK_EMAC_EN,
            );
        }
    }

    /// Configure SMI pins (MDC/MDIO) via GPIO Matrix.
    ///
    /// Uses `self.config.pins` for the GPIO numbers. MDC is output-only;
    /// MDIO is bidirectional (requires both output and input routing).
    fn configure_smi_pins(&self) {
        let mdc = self.config.pins.mdc;
        let mdio = self.config.pins.mdio;

        // SAFETY: GPIO Matrix register addresses are valid for ESP32.
        unsafe {
            // ── MDC (output only) ───────────────────────────────────
            // 1. IO_MUX → GPIO Matrix function (function 2).
            let mdc_iomux = iomux_addr_for_gpio(mdc);
            if mdc_iomux != 0 {
                let v = core::ptr::read_volatile(mdc_iomux as *const u32);
                core::ptr::write_volatile(
                    mdc_iomux as *mut u32,
                    (v & !IO_MUX_MCU_SEL_MASK) | (IO_MUX_FUNC_GPIO << IO_MUX_MCU_SEL_SHIFT),
                );
            }
            // 2. Enable output driver.
            core::ptr::write_volatile(
                (GPIO_BASE + GPIO_ENABLE_W1TS_OFFSET) as *mut u32,
                1u32 << mdc,
            );
            // 3. Route EMAC_MDC_O to this GPIO.
            let out_addr = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (mdc as usize * 4);
            core::ptr::write_volatile(
                out_addr as *mut u32,
                (EMAC_MDC_O_IDX & GPIO_FUNC_OUT_SEL_MASK) | GPIO_OEN_SEL,
            );

            // ── MDIO (bidirectional) ────────────────────────────────
            // 1. IO_MUX → GPIO Matrix function (function 2) + input enable.
            let mdio_iomux = iomux_addr_for_gpio(mdio);
            if mdio_iomux != 0 {
                let v = core::ptr::read_volatile(mdio_iomux as *const u32);
                core::ptr::write_volatile(
                    mdio_iomux as *mut u32,
                    (v & !IO_MUX_MCU_SEL_MASK)
                        | (IO_MUX_FUNC_GPIO << IO_MUX_MCU_SEL_SHIFT)
                        | IO_MUX_FUN_IE,
                );
            }
            // 2. Enable output driver.
            core::ptr::write_volatile(
                (GPIO_BASE + GPIO_ENABLE_W1TS_OFFSET) as *mut u32,
                1u32 << mdio,
            );
            // 3. Route EMAC_MDO_O output.
            let mdio_out = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (mdio as usize * 4);
            core::ptr::write_volatile(
                mdio_out as *mut u32,
                (EMAC_MDO_O_IDX & GPIO_FUNC_OUT_SEL_MASK) | GPIO_OEN_SEL,
            );
            // 4. Route EMAC_MDI_I input from this GPIO.
            let mdio_in = GPIO_BASE + GPIO_FUNC_IN_SEL_CFG_BASE + (EMAC_MDI_I_IDX as usize * 4);
            core::ptr::write_volatile(
                mdio_in as *mut u32,
                (mdio as u32 & GPIO_FUNC_IN_SEL_MASK) | GPIO_SIG_IN_SEL,
            );
        }
    }

    /// Configure fixed RMII data pins via IO_MUX (function 5).
    ///
    /// These pins are hard-wired in the ESP32 silicon and cannot
    /// be remapped: TXD0=19, TXD1=22, TX_EN=21, RXD0=25, RXD1=26,
    /// CRS_DV=27.
    fn configure_rmii_data_pins(&self) {
        // TX pins (output).
        configure_iomux_output(19); // TXD0
        configure_iomux_output(22); // TXD1
        configure_iomux_output(21); // TX_EN

        // RX pins (input).
        configure_iomux_input(25); // RXD0
        configure_iomux_input(26); // RXD1
        configure_iomux_input(27); // CRS_DV
    }

    /// Configure PHY interface registers (RMII mode and clock source).
    fn configure_phy_interface(&self) {
        // SAFETY: peripheral clock is enabled (step 1 ran already).
        unsafe {
            // Set RMII mode in EX_PHYINF_CONF.
            let conf = crate::regs::ext::read(crate::regs::ext::EX_PHYINF_CONF);
            let rmii_val = crate::regs::ext::phyinf_conf::PHY_INTF_RMII
                << crate::regs::ext::phyinf_conf::PHY_INTF_SEL_SHIFT;
            crate::regs::ext::write(
                crate::regs::ext::EX_PHYINF_CONF,
                (conf & !crate::regs::ext::phyinf_conf::PHY_INTF_SEL_MASK) | rmii_val,
            );

            // Clock source.
            match self.config.clock {
                RmiiClockConfig::External { gpio } => {
                    // External clock input on GPIO0.
                    if gpio == ClkGpio::Gpio0 {
                        // Configure GPIO0 IO_MUX for EMAC_TX_CLK (function 5, input).
                        let addr =
                            crate::regs::ext::IO_MUX_BASE + crate::regs::ext::IO_MUX_GPIO0_OFFSET;
                        let v = core::ptr::read_volatile(addr as *const u32);
                        core::ptr::write_volatile(
                            addr as *mut u32,
                            (v & !IO_MUX_MCU_SEL_MASK)
                                | (crate::regs::ext::IO_MUX_GPIO0_FUNC_EMAC_TX_CLK
                                    << IO_MUX_MCU_SEL_SHIFT)
                                | crate::regs::ext::IO_MUX_FUN_IE,
                        );
                    }

                    // EX_CLK_CTRL: ext_en=1, int_en=0.
                    let ctrl = crate::regs::ext::read(crate::regs::ext::EX_CLK_CTRL);
                    crate::regs::ext::write(
                        crate::regs::ext::EX_CLK_CTRL,
                        (ctrl | crate::regs::ext::clk_ctrl::EXT_EN)
                            & !crate::regs::ext::clk_ctrl::INT_EN,
                    );

                    // EX_OSCCLK_CONF: clk_sel=1 (external).
                    let osc = crate::regs::ext::read(crate::regs::ext::EX_OSCCLK_CONF);
                    crate::regs::ext::write(
                        crate::regs::ext::EX_OSCCLK_CONF,
                        osc | crate::regs::ext::oscclk_conf::CLK_SEL,
                    );
                }
                RmiiClockConfig::InternalApll { gpio: _ } => {
                    // Internal APLL clock (APLL must be configured externally).
                    // EX_CLK_CTRL: int_en=1, ext_en=0.
                    let ctrl = crate::regs::ext::read(crate::regs::ext::EX_CLK_CTRL);
                    crate::regs::ext::write(
                        crate::regs::ext::EX_CLK_CTRL,
                        (ctrl | crate::regs::ext::clk_ctrl::INT_EN)
                            & !crate::regs::ext::clk_ctrl::EXT_EN,
                    );

                    // EX_OSCCLK_CONF: clk_sel=0 (internal).
                    let osc = crate::regs::ext::read(crate::regs::ext::EX_OSCCLK_CONF);
                    crate::regs::ext::write(
                        crate::regs::ext::EX_OSCCLK_CONF,
                        osc & !crate::regs::ext::oscclk_conf::CLK_SEL,
                    );
                }
            }
        }
    }

    /// Enable extension clocks and power up EMAC RAM.
    fn enable_ext_clocks(&self) {
        // SAFETY: peripheral clock is enabled.
        unsafe {
            // Enable MII TX/RX clocks + main clock enable.
            let ctrl = crate::regs::ext::read(crate::regs::ext::EX_CLK_CTRL);
            crate::regs::ext::write(
                crate::regs::ext::EX_CLK_CTRL,
                ctrl | crate::regs::ext::clk_ctrl::MII_CLK_TX_EN
                    | crate::regs::ext::clk_ctrl::MII_CLK_RX_EN
                    | crate::regs::ext::clk_ctrl::CLK_EN,
            );

            // Power up EMAC RAM (clear power-down bits).
            crate::regs::ext::write(crate::regs::ext::EX_PD_SEL, 0);
        }
    }

    /// Perform DMA software reset (DMABUSMODE.SW_RESET).
    ///
    /// The bit auto-clears when the reset is complete. We poll until
    /// it clears or timeout.
    fn dma_software_reset(&self) -> Result<(), EmacError> {
        // SAFETY: peripheral clock is enabled.
        unsafe {
            crate::regs::dma::set_bits(
                crate::regs::dma::DMABUSMODE,
                crate::regs::dma::bus_mode::SW_RESET,
            );

            for _ in 0..DMA_RESET_TIMEOUT {
                let v = crate::regs::dma::read(crate::regs::dma::DMABUSMODE);
                if v & crate::regs::dma::bus_mode::SW_RESET == 0 {
                    return Ok(());
                }
                core::hint::spin_loop();
            }
        }
        Err(EmacError::Timeout)
    }

    /// Configure MAC defaults: port select, speed 100, full duplex,
    /// auto pad/CRC strip, jabber disable, watchdog disable.
    fn configure_mac_defaults(&self) {
        let cfg = crate::regs::mac::config::PORT_SELECT
            | crate::regs::mac::config::SPEED_100
            | crate::regs::mac::config::DUPLEX_FULL
            | crate::regs::mac::config::AUTO_PAD_CRC_STRIP
            | crate::regs::mac::config::JABBER_DISABLE
            | crate::regs::mac::config::WATCHDOG_DISABLE;

        // SAFETY: peripheral clock is enabled.
        unsafe {
            crate::regs::mac::write(crate::regs::mac::GMACCONFIG, cfg);

            // Frame filter: pass all multicast (for now).
            crate::regs::mac::write(
                crate::regs::mac::GMACFF,
                crate::regs::mac::frame_filter::PASS_ALL_MULTICAST,
            );

            // Clear hash tables.
            crate::regs::mac::write(crate::regs::mac::GMACHASTH, 0);
            crate::regs::mac::write(crate::regs::mac::GMACHASTL, 0);
        }
    }

    /// Configure DMA defaults: bus mode (fixed burst, AAL, ATDS,
    /// PBL=32) and operation mode (TX/RX store-and-forward).
    fn configure_dma_defaults(&self) {
        const PBL: u32 = 32;

        let bus_mode = crate::regs::dma::bus_mode::FIXED_BURST
            | crate::regs::dma::bus_mode::AAL
            | crate::regs::dma::bus_mode::USP
            | crate::regs::dma::bus_mode::ATDS
            | ((PBL << crate::regs::dma::bus_mode::PBL_SHIFT)
                & crate::regs::dma::bus_mode::PBL_MASK);

        let op_mode = crate::regs::dma::operation::TSF | crate::regs::dma::operation::RSF;

        // SAFETY: peripheral clock is enabled.
        unsafe {
            crate::regs::dma::write(crate::regs::dma::DMABUSMODE, bus_mode);
            crate::regs::dma::write(crate::regs::dma::DMAOPERATION, op_mode);

            // Disable all DMA interrupts during init.
            crate::regs::dma::write(crate::regs::dma::DMAINTENABLE, 0);

            // Clear any pending interrupt flags.
            crate::regs::dma::write(
                crate::regs::dma::DMASTATUS,
                crate::regs::dma::status::ALL_INTERRUPTS,
            );
        }
    }

    /// Program DMA descriptor list base addresses.
    fn program_dma_addresses(&self, rx_base: u32, tx_base: u32) {
        // SAFETY: peripheral clock is enabled.
        unsafe {
            crate::regs::dma::write(crate::regs::dma::DMARXBASEADDR, rx_base);
            crate::regs::dma::write(crate::regs::dma::DMATXBASEADDR, tx_base);
        }
    }

    /// Program primary MAC address into GMACADDR0H/L registers.
    fn program_mac_address(&self) {
        let m = &self.mac_address;
        let low =
            (m[0] as u32) | ((m[1] as u32) << 8) | ((m[2] as u32) << 16) | ((m[3] as u32) << 24);
        let high = (m[4] as u32) | ((m[5] as u32) << 8);

        // SAFETY: peripheral clock is enabled.
        unsafe {
            crate::regs::mac::write(crate::regs::mac::GMACADDR0L, low);
            crate::regs::mac::write(crate::regs::mac::GMACADDR0H, high);
        }
    }
}

impl<const RX: usize, const TX: usize, const BUF: usize> Default for Emac<RX, TX, BUF> {
    fn default() -> Self {
        Self::new(EmacConfig {
            clock: RmiiClockConfig::External {
                gpio: ClkGpio::Gpio0,
            },
            pins: crate::config::RmiiPins::default(),
        })
    }
}

// SAFETY: Emac can be shared between threads when properly synchronized.
unsafe impl<const RX: usize, const TX: usize, const BUF: usize> Sync for Emac<RX, TX, BUF> {}
// SAFETY: Emac can be sent between threads.
unsafe impl<const RX: usize, const TX: usize, const BUF: usize> Send for Emac<RX, TX, BUF> {}

/// Default EMAC: 10 RX, 10 TX, 1600 B buffers.
pub type EmacDefault = Emac<10, 10, 1600>;

/// Small EMAC for memory-constrained systems.
pub type EmacSmall = Emac<4, 4, 1600>;

// =============================================================================
// IO_MUX helpers (module-private)
// =============================================================================

/// Return IO_MUX register address for a given GPIO number.
///
/// Returns 0 for unsupported GPIOs. Based on ESP32 TRM Table 4-3.
fn iomux_addr_for_gpio(gpio: u8) -> usize {
    let offset: usize = match gpio {
        0 => 0x44,
        1 => 0x88,
        2 => 0x40,
        3 => 0x84,
        4 => 0x48,
        5 => 0x6C,
        6 => 0x60,
        7 => 0x64,
        8 => 0x68,
        9 => 0x54,
        10 => 0x58,
        11 => 0x5C,
        12 => 0x34,
        13 => 0x38,
        14 => 0x30,
        15 => 0x3C,
        16 => 0x4C,
        17 => 0x50,
        18 => 0x70,
        19 => 0x74,
        20 => 0x78,
        21 => 0x7C,
        22 => 0x80,
        23 => 0x8C,
        25 => 0x24,
        26 => 0x28,
        27 => 0x2C,
        32 => 0x1C,
        33 => 0x20,
        34 => 0x14,
        35 => 0x18,
        36 => 0x04,
        37 => 0x08,
        38 => 0x0C,
        39 => 0x10,
        _ => return 0,
    };
    IO_MUX_BASE + offset
}

/// Configure a GPIO as IO_MUX output for EMAC (function 5, max drive strength).
fn configure_iomux_output(gpio: u8) {
    let addr = iomux_addr_for_gpio(gpio);
    if addr == 0 {
        return;
    }
    // SAFETY: IO_MUX register addresses are valid for ESP32.
    unsafe {
        let v = core::ptr::read_volatile(addr as *const u32);
        let new = (v & !IO_MUX_MCU_SEL_MASK & !IO_MUX_FUN_IE & !IO_MUX_FUN_DRV_MASK)
            | (IO_MUX_FUNC_EMAC << IO_MUX_MCU_SEL_SHIFT)
            | (3 << 10); // max drive strength
        core::ptr::write_volatile(addr as *mut u32, new);

        // Disconnect GPIO Matrix output (use IO_MUX directly).
        let out_sel = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (gpio as usize * 4);
        core::ptr::write_volatile(out_sel as *mut u32, 256); // SIG_GPIO_OUT_IDX
    }
}

/// Configure a GPIO as IO_MUX input for EMAC (function 5, input enabled).
fn configure_iomux_input(gpio: u8) {
    let addr = iomux_addr_for_gpio(gpio);
    if addr == 0 {
        return;
    }
    // SAFETY: IO_MUX register addresses are valid for ESP32.
    unsafe {
        let v = core::ptr::read_volatile(addr as *const u32);
        let new =
            (v & !IO_MUX_MCU_SEL_MASK) | (IO_MUX_FUNC_EMAC << IO_MUX_MCU_SEL_SHIFT) | IO_MUX_FUN_IE;
        core::ptr::write_volatile(addr as *mut u32, new);

        // Disconnect GPIO Matrix output.
        let out_sel = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (gpio as usize * 4);
        core::ptr::write_volatile(out_sel as *mut u32, 256);
    }
}

// =============================================================================
// Tests (host-testable, no hardware access)
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::RmiiPins;

    /// Helper: build a test config.
    fn test_config() -> EmacConfig {
        EmacConfig {
            clock: RmiiClockConfig::External {
                gpio: ClkGpio::Gpio0,
            },
            pins: RmiiPins::default(),
        }
    }

    // ── EmacState ────────────────────────────────────────────────────

    #[test]
    fn emac_state_equality() {
        assert_eq!(EmacState::Uninitialized, EmacState::Uninitialized);
        assert_ne!(EmacState::Uninitialized, EmacState::Initialized);
        assert_ne!(EmacState::Initialized, EmacState::Running);
    }

    #[test]
    fn emac_state_debug() {
        extern crate alloc;
        let s = alloc::format!("{:?}", EmacState::Running);
        assert!(s.contains("Running"));
    }

    // ── Speed / Duplex ───────────────────────────────────────────────

    #[test]
    fn speed_enum_values() {
        assert_eq!(Speed::Mbps10, Speed::Mbps10);
        assert_ne!(Speed::Mbps10, Speed::Mbps100);
    }

    #[test]
    fn duplex_enum_values() {
        assert_eq!(Duplex::Half, Duplex::Half);
        assert_ne!(Duplex::Half, Duplex::Full);
    }

    // ── Emac struct (pure logic) ─────────────────────────────────────

    #[test]
    fn new_creates_uninitialized() {
        let emac: Emac<4, 4, 256> = Emac::new(test_config());
        assert_eq!(emac.state(), EmacState::Uninitialized);
    }

    #[test]
    fn mac_address_default_zero() {
        let emac: Emac<4, 4, 256> = Emac::new(test_config());
        assert_eq!(emac.mac_address(), [0u8; 6]);
    }

    #[test]
    fn set_mac_address_stores() {
        let mut emac: Emac<4, 4, 256> = Emac::new(test_config());
        let mac = [0x02, 0x42, 0xAC, 0x11, 0x00, 0x02];
        emac.set_mac_address(mac);
        assert_eq!(emac.mac_address(), mac);
    }

    #[test]
    fn memory_usage_matches_dma_engine() {
        let expected = DmaEngine::<10, 10, 1600>::memory_usage();
        assert_eq!(Emac::<10, 10, 1600>::memory_usage(), expected);
    }

    #[test]
    fn default_trait() {
        let emac: Emac<4, 4, 256> = Emac::default();
        assert_eq!(emac.state(), EmacState::Uninitialized);
    }

    // ── IO_MUX address lookup ────────────────────────────────────────

    #[test]
    fn iomux_known_gpios() {
        assert_eq!(iomux_addr_for_gpio(0), 0x3FF4_9044);
        assert_eq!(iomux_addr_for_gpio(18), 0x3FF4_9070);
        assert_eq!(iomux_addr_for_gpio(23), 0x3FF4_908C);
        assert_eq!(iomux_addr_for_gpio(25), 0x3FF4_9024);
    }

    #[test]
    fn iomux_unsupported_gpio_returns_zero() {
        assert_eq!(iomux_addr_for_gpio(24), 0);
        assert_eq!(iomux_addr_for_gpio(28), 0);
        assert_eq!(iomux_addr_for_gpio(255), 0);
    }
}
