// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! APLL 50 MHz clock configuration and GPIO clock output/input setup.
//!
//! The ESP32 EMAC RMII interface requires a 50 MHz reference clock.
//! It can be generated internally by the Audio PLL (APLL) or supplied
//! externally from the PHY crystal oscillator.
//!
//! ## Internal APLL mode
//!
//! 1. [`configure_apll_50mhz`] powers up APLL and programs its coefficients
//!    via ROM I2C to produce 50 MHz from the 40 MHz XTAL.
//! 2. [`configure_emac_clk_out`] sets up a GPIO (0, 16, or 17) as clock
//!    output via IO_MUX function 5 so the PHY receives 50 MHz.
//!
//! The EMAC_EXT clock path registers (int_en, clk_sel, clk_en) are
//! configured separately by [`Emac::init`](crate::emac::Emac::init)
//! via `configure_phy_interface()` and `enable_ext_clocks()`.
//!
//! ## External clock mode
//!
//! [`configure_emac_clk_in`] sets up a GPIO as clock input via IO_MUX.
//! The EMAC_EXT registers for external mode are handled by `Emac::init`.
//!
//! ## APLL/WiFi conflict
//!
//! APLL cannot coexist with WiFi/BT (ESP32 errata CLK-3.22).
//! Use external clock when Ethernet + WiFi is needed.
//!
//! ## ROM I2C details
//!
//! esp-hal does not yet expose APLL configuration (its `soc/esp32/clocks.rs`
//! has `todo!()`). We use the ROM I2C functions directly:
//! - APLL I2C block ID: `0x6D`, host ID: **3** (verified on hardware).
//! - ANA_CONF register (`0x3FF4_8030`): bit 24 = PU, bit 23 = PD.

use crate::config::ClkGpio;

// =============================================================================
// APLL ROM I2C constants
// =============================================================================

/// APLL I2C block identifier for ROM I2C functions.
const I2C_APLL: u8 = 0x6D;

/// APLL I2C host identifier (ESP32-specific, verified on hardware).
///
/// ESP-IDF headers suggest 0 or 4, but hardware testing confirmed
/// host ID 3 is correct for ESP32 APLL access.
const I2C_APLL_HOSTID: u8 = 3;

/// RTC analog configuration register address.
///
/// Contains APLL power-up (bit 24) and power-down (bit 23) controls.
/// From ESP32 SVD: `RTC_CNTL_ANA_CONF_REG`.
const ANA_CONF_REG: usize = 0x3FF4_8030;

/// APLL force power-up bit in ANA_CONF (bit 24).
const ANA_CONF_PLLA_FORCE_PU: u32 = 1 << 24;

/// APLL force power-down bit in ANA_CONF (bit 23).
const ANA_CONF_PLLA_FORCE_PD: u32 = 1 << 23;

// =============================================================================
// GPIO/IO_MUX constants
// =============================================================================

/// IO_MUX base address (ESP32).
const IO_MUX_BASE: usize = 0x3FF4_9000;

/// GPIO peripheral base address.
const GPIO_BASE: usize = 0x3FF4_4000;

/// GPIO output function select register base offset.
/// For GPIO N: `GPIO_BASE + 0x530 + N*4`.
const GPIO_FUNC_OUT_SEL_BASE: usize = GPIO_BASE + 0x530;

/// GPIO output enable set (write-1-to-set) register.
const GPIO_ENABLE_W1TS: usize = GPIO_BASE + 0x024;

/// IO_MUX MCU_SEL field mask (bits 14:12).
const MCU_SEL_MASK: u32 = 0x7 << 12;

/// IO_MUX FUN_DRV (drive strength) field mask (bits 11:10).
const FUN_DRV_MASK: u32 = 0x3 << 10;

/// IO_MUX FUN_IE (input enable) bit 9.
const FUN_IE: u32 = 1 << 9;

/// Number of spin-loop iterations to wait after APLL power-up.
///
/// Matches the firmware reference. Provides ~10-20 us settling time
/// at typical ESP32 CPU frequencies (160-240 MHz).
const APLL_POWER_UP_SPIN: u32 = 10_000;

// =============================================================================
// ROM I2C FFI
// =============================================================================

unsafe extern "C" {
    fn rom_i2c_writeReg(block: u8, block_hostid: u8, reg_add: u8, indata: u8);
    fn rom_i2c_readReg(block: u8, block_hostid: u8, reg_add: u8) -> u8;
}

/// Read an APLL register via ROM I2C.
#[inline(always)]
fn regi2c_read(reg: u8) -> u8 {
    // SAFETY: ROM I2C functions are always available on ESP32.
    unsafe { rom_i2c_readReg(I2C_APLL, I2C_APLL_HOSTID, reg) }
}

/// Write an APLL register via ROM I2C.
#[inline(always)]
fn regi2c_write(reg: u8, data: u8) {
    // SAFETY: ROM I2C functions are always available on ESP32.
    unsafe { rom_i2c_writeReg(I2C_APLL, I2C_APLL_HOSTID, reg, data) }
}

/// Masked write to an APLL register: modify bits `[msb:lsb]` to `val`.
fn apll_write_mask(reg: u8, msb: u8, lsb: u8, val: u8) {
    let old = regi2c_read(reg);
    let mask = ((1u16 << (msb - lsb + 1)) - 1) as u8;
    let new = (old & !(mask << lsb)) | ((val & mask) << lsb);
    regi2c_write(reg, new);
}

// =============================================================================
// Public API
// =============================================================================

/// Configure ESP32 APLL to output 50 MHz for EMAC RMII clock.
///
/// APLL formula: `freq = XTAL * (sdm2 + 4) / (2 * (o_div + 2))`
///
/// For 40 MHz XTAL: `50 = 40 * (6 + 4) / (2 * (2 + 2)) = 400 / 8 = 50 MHz`
///
/// This function:
/// 1. Powers up APLL via ANA_CONF register
/// 2. Configures SDM coefficients (sdm2=6, sdm1=0, sdm0=0, o_div=2)
/// 3. Runs the calibration sequence (from ESP-IDF `clk_ll_apll_set_config`)
///
/// The EMAC_EXT clock path registers (RMII mode, int_en, clk_sel) are
/// configured separately by [`Emac::init`](crate::emac::Emac::init).
///
/// # Safety
///
/// This function writes to RTC analog registers and APLL coefficients
/// via ROM I2C. The EMAC peripheral clock must be enabled before
/// calling this function.
pub fn configure_apll_50mhz() {
    // Step 1: Power up APLL
    // ANA_CONF: clear PD (bit 23), set PU (bit 24)
    unsafe {
        let ana = core::ptr::read_volatile(ANA_CONF_REG as *const u32);
        core::ptr::write_volatile(
            ANA_CONF_REG as *mut u32,
            (ana & !ANA_CONF_PLLA_FORCE_PD) | ANA_CONF_PLLA_FORCE_PU,
        );
    }
    // Wait for APLL to stabilize.
    for _ in 0..APLL_POWER_UP_SPIN {
        core::hint::spin_loop();
    }

    // Step 2: APLL coefficients -- sdm2=6, sdm1=0, sdm0=0
    apll_write_mask(7, 5, 0, 6); // sdm2
    apll_write_mask(9, 7, 0, 0); // sdm0
    apll_write_mask(8, 7, 0, 0); // sdm1

    // Step 3: Calibration sequence (from ESP-IDF clk_ll_apll_set_config)
    regi2c_write(5, 0x09);
    regi2c_write(5, 0x49);
    apll_write_mask(4, 4, 0, 2); // o_div
    regi2c_write(0, 0x0F);
    regi2c_write(0, 0x3F);
    regi2c_write(0, 0x1F);
}

/// Configure a GPIO as EMAC 50 MHz RMII clock output via IO_MUX function 5.
///
/// On ESP32, only GPIO0, GPIO16, and GPIO17 support EMAC clock output:
/// - GPIO0:  `EMAC_TX_CLK` (also boot strapping -- use with caution)
/// - GPIO16: `EMAC_CLK_OUT` (0 degree phase)
/// - GPIO17: `EMAC_CLK_OUT_180` (180 degree phase, most common for LAN8720A)
///
/// Sets IO_MUX to function 5 with maximum drive strength, disconnects
/// the GPIO Matrix (IO_MUX direct), and enables the output driver.
///
/// # Safety
///
/// Writes to IO_MUX and GPIO registers. Must be called before DMA reset.
pub fn configure_emac_clk_out(gpio: ClkGpio) {
    let io_mux_addr = io_mux_addr_for_clk_gpio(gpio);
    let gpio_num = gpio.gpio_num() as usize;

    unsafe {
        // Set IO_MUX function 5 (EMAC clock) + maximum drive strength (3).
        let val = core::ptr::read_volatile(io_mux_addr as *const u32);
        core::ptr::write_volatile(
            io_mux_addr as *mut u32,
            (val & !MCU_SEL_MASK & !FUN_DRV_MASK) | (5 << 12) | (3 << 10),
        );

        // Disconnect GPIO Matrix -- use IO_MUX directly.
        // Writing 256 (SIG_GPIO_OUT_IDX) disconnects the Matrix output.
        core::ptr::write_volatile((GPIO_FUNC_OUT_SEL_BASE + gpio_num * 4) as *mut u32, 256);

        // Enable output driver.
        core::ptr::write_volatile(GPIO_ENABLE_W1TS as *mut u32, 1u32 << gpio_num);
    }
}

/// Configure a GPIO as EMAC external 50 MHz clock input via IO_MUX.
///
/// Sets IO_MUX to function 5 with input enabled. Disconnects GPIO Matrix
/// to ensure IO_MUX is used directly.
///
/// Typically GPIO0 (`EMAC_TX_CLK` / RMII ref clock input).
///
/// # Safety
///
/// Writes to IO_MUX and GPIO registers. Must be called before DMA reset.
pub fn configure_emac_clk_in(gpio: ClkGpio) {
    let io_mux_addr = io_mux_addr_for_clk_gpio(gpio);
    let gpio_num = gpio.gpio_num() as usize;

    unsafe {
        // Set IO_MUX function 5 (EMAC clock) + input enable.
        let val = core::ptr::read_volatile(io_mux_addr as *const u32);
        core::ptr::write_volatile(
            io_mux_addr as *mut u32,
            (val & !MCU_SEL_MASK) | (5 << 12) | FUN_IE,
        );

        // Disconnect GPIO Matrix output -- use IO_MUX directly.
        core::ptr::write_volatile((GPIO_FUNC_OUT_SEL_BASE + gpio_num * 4) as *mut u32, 256);
    }
}

// =============================================================================
// Helpers
// =============================================================================

/// Return the IO_MUX register address for a clock-capable GPIO.
///
/// Based on ESP32 TRM Table 4-3:
/// - GPIO0:  offset 0x44
/// - GPIO16: offset 0x4C
/// - GPIO17: offset 0x50
const fn io_mux_addr_for_clk_gpio(gpio: ClkGpio) -> usize {
    let offset = match gpio {
        ClkGpio::Gpio0 => 0x44,
        ClkGpio::Gpio16 => 0x4C,
        ClkGpio::Gpio17 => 0x50,
    };
    IO_MUX_BASE + offset
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clk_gpio_io_mux_addresses() {
        // Verify IO_MUX offsets match the ESP32 TRM pad list.
        assert_eq!(
            io_mux_addr_for_clk_gpio(ClkGpio::Gpio0),
            0x3FF4_9044,
            "GPIO0 IO_MUX address mismatch"
        );
        assert_eq!(
            io_mux_addr_for_clk_gpio(ClkGpio::Gpio16),
            0x3FF4_904C,
            "GPIO16 IO_MUX address mismatch"
        );
        assert_eq!(
            io_mux_addr_for_clk_gpio(ClkGpio::Gpio17),
            0x3FF4_9050,
            "GPIO17 IO_MUX address mismatch"
        );
    }

    #[test]
    fn clk_gpio_numbers_match_enum() {
        assert_eq!(ClkGpio::Gpio0.gpio_num(), 0);
        assert_eq!(ClkGpio::Gpio16.gpio_num(), 16);
        assert_eq!(ClkGpio::Gpio17.gpio_num(), 17);
    }

    #[test]
    fn ana_conf_bits_no_overlap() {
        assert_eq!(
            ANA_CONF_PLLA_FORCE_PU & ANA_CONF_PLLA_FORCE_PD,
            0,
            "PU and PD bits must not overlap"
        );
    }

    #[test]
    fn ana_conf_bit_positions() {
        // PD = bit 23, PU = bit 24
        assert_eq!(ANA_CONF_PLLA_FORCE_PD, 1 << 23);
        assert_eq!(ANA_CONF_PLLA_FORCE_PU, 1 << 24);
    }

    #[test]
    fn ana_conf_register_address() {
        assert_eq!(ANA_CONF_REG, 0x3FF4_8030);
    }

    #[test]
    fn apll_constants() {
        assert_eq!(I2C_APLL, 0x6D);
        assert_eq!(I2C_APLL_HOSTID, 3);
    }

    #[test]
    fn io_mux_base_consistent_with_ext_regs() {
        assert_eq!(IO_MUX_BASE, crate::regs::ext::IO_MUX_BASE);
    }

    #[test]
    fn gpio_register_layout() {
        // GPIO_FUNC_OUT_SEL for GPIO0 should be at GPIO_BASE + 0x530
        assert_eq!(GPIO_FUNC_OUT_SEL_BASE, 0x3FF4_4530);
        // GPIO_ENABLE_W1TS should be at GPIO_BASE + 0x024
        assert_eq!(GPIO_ENABLE_W1TS, 0x3FF4_4024);
    }

    #[test]
    fn mcu_sel_mask_covers_function_5() {
        // Function 5 = 0b101, fits in 3-bit MCU_SEL field at bits 14:12
        let func5_shifted = 5u32 << 12;
        assert_eq!(func5_shifted & MCU_SEL_MASK, func5_shifted);
    }

    #[test]
    fn fun_drv_max_strength() {
        // Max drive strength = 3, shifted to bits 11:10
        let max_drv = 3u32 << 10;
        assert_eq!(max_drv & FUN_DRV_MASK, max_drv);
    }
}
