// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! MAC core register definitions.
//!
//! The MAC core handles frame transmission and reception per IEEE 802.3.
//! Base address: `0x3FF6_A000`.

#![allow(dead_code)]

// =============================================================================
// Base Address
// =============================================================================

/// MAC register block base address (ESP32).
pub const BASE: usize = 0x3FF6_A000;

// =============================================================================
// Register Offsets
// =============================================================================

/// GMAC Configuration Register.
pub const GMACCONFIG: usize = 0x00;
/// GMAC Frame Filter Register.
pub const GMACFF: usize = 0x04;
/// GMAC Hash Table High Register.
pub const GMACHASTH: usize = 0x08;
/// GMAC Hash Table Low Register.
pub const GMACHASTL: usize = 0x0C;
/// GMAC MII Address Register.
pub const GMACMIIADDR: usize = 0x10;
/// GMAC MII Data Register.
pub const GMACMIIDATA: usize = 0x14;
/// GMAC Flow Control Register.
pub const GMACFC: usize = 0x18;
/// GMAC VLAN Tag Register.
pub const GMACVLAN: usize = 0x1C;
/// GMAC Debug Register (read-only).
pub const GMACDEBUG: usize = 0x24;
/// GMAC Interrupt Status Register.
pub const GMACINTS: usize = 0x38;
/// GMAC Interrupt Mask Register.
pub const GMACINTMASK: usize = 0x3C;
/// GMAC Address 0 High Register (upper 16 bits of primary MAC).
pub const GMACADDR0H: usize = 0x40;
/// GMAC Address 0 Low Register (lower 32 bits of primary MAC).
pub const GMACADDR0L: usize = 0x44;

// =============================================================================
// GMACCONFIG bits
// =============================================================================

/// Bit-field constants for the GMAC Configuration Register.
pub mod config {
    /// Receiver Enable.
    pub const RX_ENABLE: u32 = 1 << 2;
    /// Transmitter Enable.
    pub const TX_ENABLE: u32 = 1 << 3;
    /// Automatic Pad/CRC Stripping.
    pub const AUTO_PAD_CRC_STRIP: u32 = 1 << 7;
    /// Link Up/Down (ESP32-specific).
    pub const LINK_UP: u32 = 1 << 8;
    /// Retry Disable.
    pub const RETRY_DISABLE: u32 = 1 << 9;
    /// Checksum Offload (IPC).
    pub const CHECKSUM_OFFLOAD: u32 = 1 << 10;
    /// Duplex Mode: 1 = full duplex.
    pub const DUPLEX_FULL: u32 = 1 << 11;
    /// Speed: 1 = 100 Mbps, 0 = 10 Mbps.
    pub const SPEED_100: u32 = 1 << 14;
    /// Port Select: must be 1 for MII/RMII.
    pub const PORT_SELECT: u32 = 1 << 15;
    /// Inter-Frame Gap shift (3-bit field at bits 19:17).
    pub const IFG_SHIFT: u32 = 17;
    /// Inter-Frame Gap mask.
    pub const IFG_MASK: u32 = 0x07 << 17;
    /// Jumbo Frame Enable.
    pub const JUMBO_FRAME: u32 = 1 << 20;
    /// Frame Burst Enable.
    pub const FRAME_BURST: u32 = 1 << 21;
    /// Jabber Disable.
    pub const JABBER_DISABLE: u32 = 1 << 22;
    /// Watchdog Disable.
    pub const WATCHDOG_DISABLE: u32 = 1 << 23;
}

// =============================================================================
// GMACFF bits
// =============================================================================

/// Bit-field constants for the GMAC Frame Filter Register.
pub mod frame_filter {
    /// Promiscuous Mode.
    pub const PROMISCUOUS: u32 = 1 << 0;
    /// Hash Unicast.
    pub const HASH_UNICAST: u32 = 1 << 1;
    /// Hash Multicast.
    pub const HASH_MULTICAST: u32 = 1 << 2;
    /// DA Inverse Filtering.
    pub const DA_INVERSE: u32 = 1 << 3;
    /// Pass All Multicast.
    pub const PASS_ALL_MULTICAST: u32 = 1 << 4;
    /// Disable Broadcast Frames.
    pub const DISABLE_BROADCAST: u32 = 1 << 5;
    /// Receive All.
    pub const RECEIVE_ALL: u32 = 1 << 31;
}

// =============================================================================
// GMACMIIADDR bits
// =============================================================================

/// Bit-field constants for the GMAC MII Address Register.
///
/// Also used by `mdio.rs` (which has its own private copies).
/// This module provides the complete reference.
pub mod miiaddr {
    /// Physical Layer Address shift (5-bit field at bits 15:11).
    pub const PA_SHIFT: u32 = 11;
    /// Physical Layer Address mask.
    pub const PA_MASK: u32 = 0x1F << 11;
    /// MII Register Address shift (5-bit field at bits 10:6).
    pub const GR_SHIFT: u32 = 6;
    /// MII Register Address mask.
    pub const GR_MASK: u32 = 0x1F << 6;
    /// CSR Clock Range shift (4-bit field at bits 5:2).
    pub const CR_SHIFT: u32 = 2;
    /// CSR Clock Range mask.
    pub const CR_MASK: u32 = 0x0F << 2;
    /// MII Write.
    pub const GW: u32 = 1 << 1;
    /// MII Busy.
    pub const GB: u32 = 1 << 0;
}

// =============================================================================
// GMACFC bits
// =============================================================================

/// Bit-field constants for the GMAC Flow Control Register.
pub mod flow_control {
    /// Flow Control Busy / Backpressure Activate.
    pub const FCB_BPA: u32 = 1 << 0;
    /// Transmit Flow Control Enable.
    pub const TX_ENABLE: u32 = 1 << 1;
    /// Receive Flow Control Enable.
    pub const RX_ENABLE: u32 = 1 << 2;
    /// Unicast PAUSE Frame Detect.
    pub const UNICAST_PAUSE: u32 = 1 << 3;
    /// PAUSE Low Threshold shift (2-bit field at bits 5:4).
    pub const PLT_SHIFT: u32 = 4;
    /// PAUSE Low Threshold mask.
    pub const PLT_MASK: u32 = 0x03 << 4;
    /// Zero-Quanta PAUSE Disable.
    pub const ZERO_QUANTA_DISABLE: u32 = 1 << 7;
    /// PAUSE Time shift (16-bit field at bits 31:16).
    pub const PT_SHIFT: u32 = 16;
    /// PAUSE Time mask.
    pub const PT_MASK: u32 = 0xFFFF << 16;
}

// =============================================================================
// Register access helpers
// =============================================================================

/// Read a MAC register at `offset` from BASE.
///
/// # Safety
/// Caller must ensure the EMAC peripheral clock is enabled and
/// `offset` is a valid register offset within this block.
#[inline(always)]
pub unsafe fn read(offset: usize) -> u32 {
    // SAFETY: caller guarantees address validity.
    core::ptr::read_volatile((BASE + offset) as *const u32)
}

/// Write a MAC register at `offset` from BASE.
///
/// # Safety
/// Caller must ensure the EMAC peripheral clock is enabled and
/// `offset` is a valid register offset within this block.
#[inline(always)]
pub unsafe fn write(offset: usize, val: u32) {
    // SAFETY: caller guarantees address validity.
    core::ptr::write_volatile((BASE + offset) as *mut u32, val);
}

/// Read-modify-write: set bits in a MAC register.
///
/// # Safety
/// Same requirements as [`read`] and [`write`].
#[inline(always)]
pub unsafe fn set_bits(offset: usize, bits: u32) {
    let val = read(offset);
    write(offset, val | bits);
}

/// Read-modify-write: clear bits in a MAC register.
///
/// # Safety
/// Same requirements as [`read`] and [`write`].
#[inline(always)]
pub unsafe fn clear_bits(offset: usize, bits: u32) {
    let val = read(offset);
    write(offset, val & !bits);
}

// =============================================================================
// Composite operations (formerly ph_esp32_mac::unsafe_registers::MacRegs)
// =============================================================================

/// Read the GMAC Configuration register.
#[inline(always)]
pub fn config() -> u32 {
    // SAFETY: GMACCONFIG is a known-valid 32-bit memory-mapped register.
    unsafe { read(GMACCONFIG) }
}

/// Write the GMAC Configuration register.
#[inline(always)]
pub fn set_config(val: u32) {
    // SAFETY: GMACCONFIG is a known-valid 32-bit memory-mapped register.
    unsafe { write(GMACCONFIG, val) }
}

/// Write the GMAC Frame Filter register.
#[inline(always)]
pub fn set_frame_filter(val: u32) {
    // SAFETY: GMACFF is a known-valid 32-bit memory-mapped register.
    unsafe { write(GMACFF, val) }
}

/// Toggle promiscuous mode in the GMAC Frame Filter register (`GMACFF`,
/// bit 0 — `PR`).
///
/// When promiscuous mode is on the MAC forwards **every** frame to the
/// host descriptor ring regardless of the destination address — bypass
/// of the unicast filter populated by [`set_mac_address`], the hash
/// table, and the broadcast/multicast bypass bits.
///
/// Performed as a strict read-modify-write so the other filter bits
/// (`HASH_UNICAST`, `HASH_MULTICAST`, `DA_INVERSE`, `PASS_ALL_MULTICAST`,
/// `DISABLE_BROADCAST`, `RECEIVE_ALL`, the `PCF`/`SAF`/`SAIF` fields,
/// etc.) survive untouched. A naive `set_frame_filter(PR)` would clear
/// them all, which is rarely what callers want.
///
/// Intended use cases:
///
/// - **PHY loopback testing**: in loopback the destination address on
///   every frame equals our source address. Unicast filtering can still
///   admit those frames if `GMACADDR0H.AE` is set correctly, but
///   promiscuous mode removes that dependency entirely and matches the
///   same datapath a wire-side echo configuration would exercise.
/// - **Raw-L2 sniffer / monitor** apps that want to see frames not
///   addressed to the local MAC.
/// - **L2 stress / throughput measurement** where the desired test
///   traffic uses MACs that wouldn't pass the unicast filter.
///
/// Not intended for production stack use; embassy-net's normal mode
/// relies on the unicast filter to keep the descriptor ring quiet.
///
/// # Precondition
///
/// Touches a memory-mapped register at the GMAC base. The EMAC peripheral
/// clock must be enabled before this is called — `Emac::init` brings the
/// clock up, so the typical call sequence is `Emac::init(...)` followed
/// by `regs::mac::set_promiscuous(true)` once the driver is ready. The
/// helper is `pub` and `safe` so calling it before the clock is on will
/// not be caught by the type system; the resulting access will fault
/// (bus error) at runtime. The precondition is shared with every other
/// `regs::mac::*` helper in this module.
#[inline]
pub fn set_promiscuous(enable: bool) {
    // SAFETY: GMACFF is a known-valid 32-bit memory-mapped register;
    // an RMW preserves the other filter bits. See the module-level
    // notes on `set_mac_address` for the broader address-filter
    // sequencing constraints — those apply to ADDR0H/L, not to GMACFF,
    // so there is no ordering interaction with this helper.
    unsafe {
        if enable {
            set_bits(GMACFF, frame_filter::PROMISCUOUS);
        } else {
            clear_bits(GMACFF, frame_filter::PROMISCUOUS);
        }
    }
}

/// Write the 64-bit hash filter table (low → `GMACHASTL`, high → `GMACHASTH`).
#[inline(always)]
pub fn set_hash_table(value: u64) {
    // SAFETY: both registers are known-valid 32-bit memory-mapped registers.
    unsafe {
        write(GMACHASTL, value as u32);
        write(GMACHASTH, (value >> 32) as u32);
    }
}

/// Apply the 100 Mbps / 10 Mbps speed setting (RMW on `GMACCONFIG.FES`).
#[inline(always)]
pub fn set_speed_100mbps(is_100: bool) {
    // SAFETY: GMACCONFIG is a known-valid 32-bit memory-mapped register.
    unsafe {
        if is_100 {
            set_bits(GMACCONFIG, config::SPEED_100);
        } else {
            clear_bits(GMACCONFIG, config::SPEED_100);
        }
    }
}

/// Apply the full / half duplex setting (RMW on `GMACCONFIG.DM`).
#[inline(always)]
pub fn set_duplex_full(full: bool) {
    // SAFETY: GMACCONFIG is a known-valid 32-bit memory-mapped register.
    unsafe {
        if full {
            set_bits(GMACCONFIG, config::DUPLEX_FULL);
        } else {
            clear_bits(GMACCONFIG, config::DUPLEX_FULL);
        }
    }
}

/// Programme the primary MAC address into ADDR0H / ADDR0L (with AE bit).
///
/// The Synopsys DesignWare GMAC core latches the filter address on the
/// write to **ADDR0L** (the low half). Therefore the high half (with the
/// `AE` bit) MUST be written first, then the low half. Writing in the
/// opposite order leaves the internal filter latched with the stale
/// reset/efuse value while register read-back happily shows the new
/// address — and unicast RX dies silently.
pub fn set_mac_address(addr: &[u8; 6]) {
    let high = (addr[4] as u32) | ((addr[5] as u32) << 8) | (1u32 << 31);
    let low = (addr[0] as u32)
        | ((addr[1] as u32) << 8)
        | ((addr[2] as u32) << 16)
        | ((addr[3] as u32) << 24);
    // SAFETY: ADDR0H/ADDR0L are valid 32-bit MAC registers; caller has
    // already enabled the EMAC peripheral clock by the time `init()`
    // reaches this step.
    unsafe {
        write(GMACADDR0H, high);
        write(GMACADDR0L, low);
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_address() {
        assert_eq!(BASE, 0x3FF6_A000);
    }

    #[test]
    fn register_offsets_within_block() {
        // MAC register block is 0x1000 (4 KiB)
        let offsets = [
            GMACCONFIG,
            GMACFF,
            GMACHASTH,
            GMACHASTL,
            GMACMIIADDR,
            GMACMIIDATA,
            GMACFC,
            GMACVLAN,
            GMACDEBUG,
            GMACINTS,
            GMACINTMASK,
            GMACADDR0H,
            GMACADDR0L,
        ];
        for off in offsets {
            assert!(off < 0x1000, "offset {:#x} exceeds MAC block size", off);
        }
    }

    #[test]
    fn config_bits_no_overlap() {
        let bits = [
            config::RX_ENABLE,
            config::TX_ENABLE,
            config::RETRY_DISABLE,
            config::CHECKSUM_OFFLOAD,
            config::DUPLEX_FULL,
            config::SPEED_100,
            config::PORT_SELECT,
            config::JUMBO_FRAME,
            config::FRAME_BURST,
            config::JABBER_DISABLE,
            config::WATCHDOG_DISABLE,
            config::LINK_UP,
            config::AUTO_PAD_CRC_STRIP,
        ];
        for i in 0..bits.len() {
            for j in (i + 1)..bits.len() {
                assert_eq!(
                    bits[i] & bits[j],
                    0,
                    "config bits {:#x} and {:#x} overlap",
                    bits[i],
                    bits[j]
                );
            }
        }
    }

    #[test]
    fn miiaddr_fields_no_overlap() {
        // Single-bit fields
        assert_eq!(miiaddr::GB & miiaddr::GW, 0);
        // Mask fields should not overlap each other
        assert_eq!(miiaddr::CR_MASK & miiaddr::GR_MASK, 0);
        assert_eq!(miiaddr::GR_MASK & miiaddr::PA_MASK, 0);
        assert_eq!(miiaddr::CR_MASK & miiaddr::PA_MASK, 0);
    }

    #[test]
    fn flow_control_bits_no_overlap() {
        let bits = [
            flow_control::FCB_BPA,
            flow_control::TX_ENABLE,
            flow_control::RX_ENABLE,
            flow_control::UNICAST_PAUSE,
            flow_control::ZERO_QUANTA_DISABLE,
        ];
        for i in 0..bits.len() {
            for j in (i + 1)..bits.len() {
                assert_eq!(
                    bits[i] & bits[j],
                    0,
                    "flow_control bits {:#x} and {:#x} overlap",
                    bits[i],
                    bits[j]
                );
            }
        }
    }

    #[test]
    fn frame_filter_bits_no_overlap() {
        let bits = [
            frame_filter::PROMISCUOUS,
            frame_filter::HASH_UNICAST,
            frame_filter::HASH_MULTICAST,
            frame_filter::DA_INVERSE,
            frame_filter::PASS_ALL_MULTICAST,
            frame_filter::DISABLE_BROADCAST,
            frame_filter::RECEIVE_ALL,
        ];
        for i in 0..bits.len() {
            for j in (i + 1)..bits.len() {
                assert_eq!(
                    bits[i] & bits[j],
                    0,
                    "frame_filter bits {:#x} and {:#x} overlap",
                    bits[i],
                    bits[j]
                );
            }
        }
    }
}
