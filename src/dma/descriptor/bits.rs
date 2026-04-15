// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! DMA descriptor bit field constants.
//!
//! Based on ESP32 TRM Chapter 10 and IEEE 802.3.

#![allow(dead_code)]

// =============================================================================
// RDES0 (RX Descriptor Word 0) — Status
// =============================================================================

/// RX Descriptor Word 0 bit field constants.
pub mod rdes0 {
    /// CRC Error — frame has CRC error.
    pub const CRC_ERR: u32 = 1 << 1;
    /// Dribble Bit Error — non-integer multiple of 8 bits.
    pub const DRIBBLE_ERR: u32 = 1 << 2;
    /// Receive Error — error reported by PHY (RX_ER signal).
    pub const RX_ERR: u32 = 1 << 3;
    /// Receive Watchdog Timeout — frame truncated.
    pub const RX_WATCHDOG: u32 = 1 << 4;
    /// Frame Type — 1 = Ethernet frame (length/type > 0x600).
    pub const FRAME_TYPE: u32 = 1 << 5;
    /// Late Collision — collision after 64 bytes.
    pub const LATE_COLLISION: u32 = 1 << 6;
    /// Last Descriptor — last descriptor for the frame.
    pub const LAST_DESC: u32 = 1 << 8;
    /// First Descriptor — first descriptor for the frame.
    pub const FIRST_DESC: u32 = 1 << 9;
    /// Overflow Error — DMA buffer overflow.
    pub const OVERFLOW_ERR: u32 = 1 << 11;
    /// Length Error — actual length doesn't match length/type field.
    pub const LENGTH_ERR: u32 = 1 << 12;
    /// Descriptor Error — descriptor not available or bus error.
    pub const DESC_ERR: u32 = 1 << 14;
    /// Error Summary — logical OR of error bits.
    pub const ERR_SUMMARY: u32 = 1 << 15;
    /// Frame Length shift (14 bits, starting at bit 16).
    pub const FRAME_LEN_SHIFT: u32 = 16;
    /// Frame Length mask.
    pub const FRAME_LEN_MASK: u32 = 0x3FFF << 16;
    /// Destination Address Filter Fail.
    pub const DA_FILTER_FAIL: u32 = 1 << 30;
    /// OWN — when set, descriptor is owned by DMA; when clear, by CPU.
    pub const OWN: u32 = 1 << 31;

    /// All possible RX error bits.
    pub const ALL_ERRORS: u32 = CRC_ERR
        | DRIBBLE_ERR
        | RX_ERR
        | RX_WATCHDOG
        | LATE_COLLISION
        | OVERFLOW_ERR
        | LENGTH_ERR
        | DESC_ERR;
}

// =============================================================================
// RDES1 (RX Descriptor Word 1) — Control
// =============================================================================

/// RX Descriptor Word 1 bit field constants.
pub mod rdes1 {
    /// RX Buffer 1 Size mask (13 bits).
    pub const BUFFER1_SIZE_MASK: u32 = 0x1FFF;
    /// RX Buffer 1 Size shift.
    pub const BUFFER1_SIZE_SHIFT: u32 = 0;
    /// Second Address Chained — buffer2 contains next descriptor address.
    pub const SECOND_ADDR_CHAINED: u32 = 1 << 14;
    /// Receive End of Ring — last descriptor in the ring.
    pub const RX_END_OF_RING: u32 = 1 << 15;
    /// Disable Interrupt on Completion.
    pub const DISABLE_IRQ: u32 = 1 << 31;
}

// =============================================================================
// TDES0 (TX Descriptor Word 0) — Status / Control
// =============================================================================

/// TX Descriptor Word 0 bit field constants.
pub mod tdes0 {
    /// Underflow Error — TX FIFO underflow during frame transmission.
    pub const UNDERFLOW_ERR: u32 = 1 << 1;
    /// Excessive Deferral — deferred for more than 24 288 bit times.
    pub const EXCESSIVE_DEFERRAL: u32 = 1 << 2;
    /// Collision Count shift (4 bits).
    pub const COLLISION_COUNT_SHIFT: u32 = 3;
    /// Collision Count mask.
    pub const COLLISION_COUNT_MASK: u32 = 0xF << 3;
    /// Excessive Collision — more than 16 collisions.
    pub const EXCESSIVE_COLLISION: u32 = 1 << 8;
    /// Late Collision — collision after 64 byte times.
    pub const LATE_COLLISION: u32 = 1 << 9;
    /// No Carrier — carrier sense not asserted.
    pub const NO_CARRIER: u32 = 1 << 10;
    /// Loss of Carrier — carrier lost during transmission.
    pub const LOSS_OF_CARRIER: u32 = 1 << 11;
    /// IP Payload Error — checksum error in payload.
    pub const IP_PAYLOAD_ERR: u32 = 1 << 12;
    /// Jabber Timeout — transmission continued beyond 2048 bytes.
    pub const JABBER_TIMEOUT: u32 = 1 << 14;
    /// Error Summary — logical OR of all error bits.
    pub const ERR_SUMMARY: u32 = 1 << 15;
    /// IP Header Error — checksum error in IP header.
    pub const IP_HEADER_ERR: u32 = 1 << 16;
    /// Second Address Chained — buffer2 contains next descriptor address.
    pub const SECOND_ADDR_CHAINED: u32 = 1 << 20;
    /// Transmit End of Ring — last descriptor in the ring.
    pub const TX_END_OF_RING: u32 = 1 << 21;
    /// Checksum Insertion Control shift (2 bits).
    pub const CHECKSUM_INSERT_SHIFT: u32 = 22;
    /// Checksum Insertion Control mask.
    pub const CHECKSUM_INSERT_MASK: u32 = 0x3 << 22;
    /// First Segment — buffer contains first segment of frame.
    pub const FIRST_SEGMENT: u32 = 1 << 28;
    /// Last Segment — buffer contains last segment of frame.
    pub const LAST_SEGMENT: u32 = 1 << 29;
    /// Interrupt on Completion — generate interrupt when complete.
    pub const INTERRUPT_ON_COMPLETE: u32 = 1 << 30;
    /// OWN — when set, descriptor is owned by DMA; when clear, by CPU.
    pub const OWN: u32 = 1 << 31;

    /// All possible TX error bits.
    pub const ALL_ERRORS: u32 = UNDERFLOW_ERR
        | EXCESSIVE_DEFERRAL
        | EXCESSIVE_COLLISION
        | LATE_COLLISION
        | NO_CARRIER
        | LOSS_OF_CARRIER
        | IP_PAYLOAD_ERR
        | JABBER_TIMEOUT
        | IP_HEADER_ERR;
}

// =============================================================================
// TDES1 (TX Descriptor Word 1) — Buffer Sizes
// =============================================================================

/// TX Descriptor Word 1 bit field constants.
pub mod tdes1 {
    /// TX Buffer 1 Size mask (13 bits).
    pub const BUFFER1_SIZE_MASK: u32 = 0x1FFF;
    /// TX Buffer 1 Size shift.
    pub const BUFFER1_SIZE_SHIFT: u32 = 0;
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rdes0_own_is_bit_31() {
        assert_eq!(rdes0::OWN, 1 << 31);
    }

    #[test]
    fn tdes0_own_is_bit_31() {
        assert_eq!(tdes0::OWN, 1 << 31);
    }

    #[test]
    fn rdes0_frame_len_mask_covers_bits_16_to_29() {
        assert_eq!(rdes0::FRAME_LEN_MASK, 0x3FFF_0000);
    }

    #[test]
    fn tdes0_first_segment_is_bit_28() {
        assert_eq!(tdes0::FIRST_SEGMENT, 1 << 28);
    }

    #[test]
    fn tdes0_last_segment_is_bit_29() {
        assert_eq!(tdes0::LAST_SEGMENT, 1 << 29);
    }

    #[test]
    fn tdes1_buffer1_size_mask_is_13_bits() {
        assert_eq!(tdes1::BUFFER1_SIZE_MASK, 0x1FFF);
    }

    #[test]
    fn rdes1_buffer1_size_mask_is_13_bits() {
        assert_eq!(rdes1::BUFFER1_SIZE_MASK, 0x1FFF);
    }

    #[test]
    fn rdes1_second_addr_chained_is_bit_14() {
        assert_eq!(rdes1::SECOND_ADDR_CHAINED, 1 << 14);
    }

    #[test]
    fn tdes0_second_addr_chained_is_bit_20() {
        assert_eq!(tdes0::SECOND_ADDR_CHAINED, 1 << 20);
    }

    #[test]
    fn tdes0_all_errors_covers_expected_bits() {
        // Each error bit must be included.
        assert!(tdes0::ALL_ERRORS & tdes0::UNDERFLOW_ERR != 0);
        assert!(tdes0::ALL_ERRORS & tdes0::EXCESSIVE_DEFERRAL != 0);
        assert!(tdes0::ALL_ERRORS & tdes0::EXCESSIVE_COLLISION != 0);
        assert!(tdes0::ALL_ERRORS & tdes0::LATE_COLLISION != 0);
        assert!(tdes0::ALL_ERRORS & tdes0::NO_CARRIER != 0);
        assert!(tdes0::ALL_ERRORS & tdes0::LOSS_OF_CARRIER != 0);
        assert!(tdes0::ALL_ERRORS & tdes0::IP_PAYLOAD_ERR != 0);
        assert!(tdes0::ALL_ERRORS & tdes0::JABBER_TIMEOUT != 0);
        assert!(tdes0::ALL_ERRORS & tdes0::IP_HEADER_ERR != 0);
    }

    #[test]
    fn rdes0_all_errors_covers_expected_bits() {
        assert!(rdes0::ALL_ERRORS & rdes0::CRC_ERR != 0);
        assert!(rdes0::ALL_ERRORS & rdes0::DRIBBLE_ERR != 0);
        assert!(rdes0::ALL_ERRORS & rdes0::RX_ERR != 0);
        assert!(rdes0::ALL_ERRORS & rdes0::RX_WATCHDOG != 0);
        assert!(rdes0::ALL_ERRORS & rdes0::LATE_COLLISION != 0);
        assert!(rdes0::ALL_ERRORS & rdes0::OVERFLOW_ERR != 0);
        assert!(rdes0::ALL_ERRORS & rdes0::LENGTH_ERR != 0);
        assert!(rdes0::ALL_ERRORS & rdes0::DESC_ERR != 0);
    }
}
