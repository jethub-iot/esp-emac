// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! TX and RX DMA descriptor structures.
//!
//! Each descriptor is 4 x u32 (16 bytes) in the simplified ESP32 layout:
//!
//! | Word | TX (TDES)          | RX (RDES)          |
//! |------|--------------------|--------------------|
//! | 0    | Status / Control   | Status             |
//! | 1    | Buffer size, flags | Buffer size, flags |
//! | 2    | Buffer pointer     | Buffer pointer     |
//! | 3    | Next descriptor    | Next descriptor    |
//!
//! The OWN bit (bit 31 of word 0) governs ownership: when set the DMA
//! engine owns the descriptor; when clear the CPU may access it.

pub mod bits;

use bits::{rdes0, rdes1, tdes0, tdes1};

// =============================================================================
// VolatileCell
// =============================================================================

/// Volatile cell wrapper for DMA descriptor fields.
///
/// Prevents the compiler from reordering or caching register-like memory
/// accesses. All reads and writes go through `core::ptr::{read,write}_volatile`.
#[repr(transparent)]
pub struct VolatileCell<T: Copy> {
    value: core::cell::UnsafeCell<T>,
}

// SAFETY: DMA descriptors are accessed from ISR context and main context.
// Volatile access + OWN-bit protocol ensures correctness.
unsafe impl<T: Copy> Sync for VolatileCell<T> {}

impl<T: Copy> VolatileCell<T> {
    /// Create a new volatile cell with the given initial value.
    #[inline(always)]
    pub const fn new(value: T) -> Self {
        Self {
            value: core::cell::UnsafeCell::new(value),
        }
    }

    /// Read the value (volatile read).
    #[inline(always)]
    pub fn get(&self) -> T {
        // SAFETY: Volatile access to a valid UnsafeCell-backed pointer.
        unsafe { core::ptr::read_volatile(self.value.get()) }
    }

    /// Write a value (volatile write).
    #[inline(always)]
    pub fn set(&self, value: T) {
        // SAFETY: Volatile access to a valid UnsafeCell-backed pointer.
        unsafe { core::ptr::write_volatile(self.value.get(), value) }
    }

    /// Update the value using a function (read-modify-write).
    #[inline(always)]
    pub fn update<F>(&self, f: F)
    where
        F: FnOnce(T) -> T,
    {
        let old = self.get();
        self.set(f(old));
    }
}

impl<T: Copy + Default> Default for VolatileCell<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}

// =============================================================================
// TX Descriptor
// =============================================================================

/// TX DMA descriptor (16 bytes, chained mode).
///
/// The CPU prepares descriptors and sets the OWN bit to hand them to the
/// DMA engine for transmission. After transmission completes the DMA
/// clears OWN and writes back status bits.
#[repr(C, align(4))]
pub struct TxDescriptor {
    /// TDES0: Status and control bits (OWN, first/last segment, etc.).
    tdes0: VolatileCell<u32>,
    /// TDES1: Buffer 1 size and control flags.
    tdes1: VolatileCell<u32>,
    /// TDES2: Buffer 1 address.
    buffer_addr: VolatileCell<u32>,
    /// TDES3: Next descriptor address (chained mode).
    next_desc_addr: VolatileCell<u32>,
}

#[allow(dead_code)]
impl TxDescriptor {
    /// Descriptor size in bytes.
    pub const SIZE: usize = 16;

    /// Create a new zeroed TX descriptor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            tdes0: VolatileCell::new(0),
            tdes1: VolatileCell::new(0),
            buffer_addr: VolatileCell::new(0),
            next_desc_addr: VolatileCell::new(0),
        }
    }

    /// Initialize descriptor for chained mode.
    ///
    /// Sets the buffer pointer, next-descriptor pointer, and the
    /// `SECOND_ADDR_CHAINED` flag. The descriptor is left CPU-owned.
    pub fn setup_chained(&self, buffer: *const u8, next_desc: *const TxDescriptor) {
        self.buffer_addr.set(buffer as u32);
        self.next_desc_addr.set(next_desc as u32);
        self.tdes0.set(tdes0::SECOND_ADDR_CHAINED);
        self.tdes1.set(0);
    }

    /// Check if DMA owns this descriptor.
    #[inline(always)]
    #[must_use]
    pub fn is_owned(&self) -> bool {
        (self.tdes0.get() & tdes0::OWN) != 0
    }

    /// Give ownership to DMA.
    #[inline(always)]
    pub fn set_owned(&self) {
        self.tdes0.update(|v| v | tdes0::OWN);
    }

    /// Take ownership from DMA.
    #[inline(always)]
    pub fn clear_owned(&self) {
        self.tdes0.update(|v| v & !tdes0::OWN);
    }

    /// Prepare descriptor for transmission with segment flags.
    ///
    /// Sets the buffer length and first/last segment flags.
    /// Does **not** set the OWN bit — call [`set_owned`](Self::set_owned)
    /// afterwards to submit to DMA.
    pub fn prepare(&self, len: usize, first: bool, last: bool) {
        let mut flags = tdes0::SECOND_ADDR_CHAINED;

        if first {
            flags |= tdes0::FIRST_SEGMENT;
        }
        if last {
            flags |= tdes0::LAST_SEGMENT | tdes0::INTERRUPT_ON_COMPLETE;
        }

        self.tdes1.set((len as u32) & tdes1::BUFFER1_SIZE_MASK);
        self.tdes0.set(flags);
    }

    /// Prepare and submit to DMA in one operation.
    pub fn prepare_and_submit(&self, len: usize, first: bool, last: bool) {
        self.prepare(len, first, last);
        self.set_owned();
    }

    /// Check if transmission had errors (error summary bit).
    #[inline(always)]
    #[must_use]
    pub fn has_error(&self) -> bool {
        (self.tdes0.get() & tdes0::ERR_SUMMARY) != 0
    }

    /// Get all error flags from TDES0.
    #[inline(always)]
    #[must_use]
    pub fn error_flags(&self) -> u32 {
        self.tdes0.get() & tdes0::ALL_ERRORS
    }

    /// Get buffer address (TDES2).
    #[inline(always)]
    #[must_use]
    pub fn buffer_addr(&self) -> u32 {
        self.buffer_addr.get()
    }

    /// Get next descriptor address (TDES3, chained mode).
    #[inline(always)]
    #[must_use]
    pub fn next_desc_addr(&self) -> u32 {
        self.next_desc_addr.get()
    }

    /// Reset descriptor to initial state, preserving the chain pointer.
    pub fn reset(&self) {
        let next = self.next_desc_addr.get();
        self.tdes0.set(tdes0::SECOND_ADDR_CHAINED);
        self.tdes1.set(0);
        self.next_desc_addr.set(next);
    }

    /// Raw TDES0 value (for debugging / tests).
    #[inline(always)]
    #[must_use]
    pub fn raw_tdes0(&self) -> u32 {
        self.tdes0.get()
    }

    /// Raw TDES1 value (for debugging / tests).
    #[inline(always)]
    #[must_use]
    pub fn raw_tdes1(&self) -> u32 {
        self.tdes1.get()
    }
}

impl Default for TxDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: TxDescriptor uses volatile cells for all DMA-accessed fields.
unsafe impl Sync for TxDescriptor {}
// SAFETY: TxDescriptor can be sent between threads.
unsafe impl Send for TxDescriptor {}

// =============================================================================
// RX Descriptor
// =============================================================================

/// RX DMA descriptor (16 bytes, chained mode).
///
/// The DMA engine fills these descriptors with received frame data and
/// writes back status bits. The CPU reads the status, processes the
/// frame, then returns the descriptor to DMA via [`recycle`](Self::recycle).
#[repr(C, align(4))]
pub struct RxDescriptor {
    /// RDES0: Status bits (OWN, first/last, frame length, errors).
    rdes0: VolatileCell<u32>,
    /// RDES1: Buffer 1 size and control flags.
    rdes1: VolatileCell<u32>,
    /// RDES2: Buffer 1 address.
    buffer_addr: VolatileCell<u32>,
    /// RDES3: Next descriptor address (chained mode).
    next_desc_addr: VolatileCell<u32>,
}

#[allow(dead_code)]
impl RxDescriptor {
    /// Descriptor size in bytes.
    pub const SIZE: usize = 16;

    /// Create a new zeroed RX descriptor. Call [`setup_chained`](Self::setup_chained) before use.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            rdes0: VolatileCell::new(0),
            rdes1: VolatileCell::new(0),
            buffer_addr: VolatileCell::new(0),
            next_desc_addr: VolatileCell::new(0),
        }
    }

    /// Configure descriptor in chained mode and give to DMA.
    ///
    /// Sets the buffer pointer, buffer size, next-descriptor pointer,
    /// the `SECOND_ADDR_CHAINED` flag, and the OWN bit.
    pub fn setup_chained(
        &self,
        buffer: *mut u8,
        buffer_size: usize,
        next_desc: *const RxDescriptor,
    ) {
        self.buffer_addr.set(buffer as u32);
        self.next_desc_addr.set(next_desc as u32);
        self.rdes1
            .set(rdes1::SECOND_ADDR_CHAINED | ((buffer_size as u32) & rdes1::BUFFER1_SIZE_MASK));
        // Give ownership to DMA.
        self.rdes0.set(rdes0::OWN);
    }

    /// Check if DMA owns this descriptor.
    #[inline(always)]
    #[must_use]
    pub fn is_owned(&self) -> bool {
        (self.rdes0.get() & rdes0::OWN) != 0
    }

    /// Give ownership to DMA.
    #[inline(always)]
    pub fn set_owned(&self) {
        self.rdes0.set(rdes0::OWN);
    }

    /// Take ownership from DMA.
    #[inline(always)]
    pub fn clear_owned(&self) {
        self.rdes0.update(|v| v & !rdes0::OWN);
    }

    /// First descriptor of a frame.
    #[inline(always)]
    #[must_use]
    pub fn is_first(&self) -> bool {
        (self.rdes0.get() & rdes0::FIRST_DESC) != 0
    }

    /// Last descriptor of a frame.
    #[inline(always)]
    #[must_use]
    pub fn is_last(&self) -> bool {
        (self.rdes0.get() & rdes0::LAST_DESC) != 0
    }

    /// Complete frame in a single descriptor (both first and last).
    #[inline(always)]
    #[must_use]
    pub fn is_complete_frame(&self) -> bool {
        let status = self.rdes0.get();
        (status & (rdes0::FIRST_DESC | rdes0::LAST_DESC)) == (rdes0::FIRST_DESC | rdes0::LAST_DESC)
    }

    /// Check if the error summary bit is set.
    #[inline(always)]
    #[must_use]
    pub fn has_error(&self) -> bool {
        (self.rdes0.get() & rdes0::ERR_SUMMARY) != 0
    }

    /// Raw error flags from RDES0.
    #[inline(always)]
    #[must_use]
    pub fn error_flags(&self) -> u32 {
        self.rdes0.get() & rdes0::ALL_ERRORS
    }

    /// Frame length including CRC (valid on last descriptor).
    #[inline(always)]
    #[must_use]
    pub fn frame_length(&self) -> usize {
        ((self.rdes0.get() & rdes0::FRAME_LEN_MASK) >> rdes0::FRAME_LEN_SHIFT) as usize
    }

    /// Frame length excluding the 4-byte CRC.
    #[inline(always)]
    #[must_use]
    pub fn payload_length(&self) -> usize {
        self.frame_length().saturating_sub(4)
    }

    /// Buffer address (RDES2).
    #[inline(always)]
    #[must_use]
    pub fn buffer_addr(&self) -> u32 {
        self.buffer_addr.get()
    }

    /// Next descriptor address (RDES3, chained mode).
    #[inline(always)]
    #[must_use]
    pub fn next_desc_addr(&self) -> u32 {
        self.next_desc_addr.get()
    }

    /// Configured buffer size from RDES1.
    #[inline(always)]
    #[must_use]
    pub fn buffer_size(&self) -> usize {
        (self.rdes1.get() & rdes1::BUFFER1_SIZE_MASK) as usize
    }

    /// Clear status and return the descriptor to DMA for reuse.
    pub fn recycle(&self) {
        self.rdes0.set(rdes0::OWN);
    }

    /// Raw RDES0 value (for debugging / tests).
    #[inline(always)]
    #[must_use]
    pub fn raw_rdes0(&self) -> u32 {
        self.rdes0.get()
    }

    /// Raw RDES1 value (for debugging / tests).
    #[inline(always)]
    #[must_use]
    pub fn raw_rdes1(&self) -> u32 {
        self.rdes1.get()
    }
}

impl Default for RxDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: RxDescriptor uses volatile cells for all DMA-accessed fields.
unsafe impl Sync for RxDescriptor {}
// SAFETY: RxDescriptor can be sent between threads.
unsafe impl Send for RxDescriptor {}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // VolatileCell Tests
    // =========================================================================

    #[test]
    fn volatile_cell_new() {
        let cell = VolatileCell::new(42u32);
        assert_eq!(cell.get(), 42);
    }

    #[test]
    fn volatile_cell_get_set() {
        let cell = VolatileCell::new(0u32);
        assert_eq!(cell.get(), 0);
        cell.set(0xDEAD_BEEF);
        assert_eq!(cell.get(), 0xDEAD_BEEF);
    }

    #[test]
    fn volatile_cell_update() {
        let cell = VolatileCell::new(0x0000_00FFu32);
        cell.update(|v| v | 0xFF00_0000);
        assert_eq!(cell.get(), 0xFF00_00FF);
    }

    #[test]
    fn volatile_cell_default() {
        let cell = VolatileCell::<u32>::default();
        assert_eq!(cell.get(), 0);
    }

    // =========================================================================
    // TX Descriptor Layout Tests
    // =========================================================================

    #[test]
    fn tx_descriptor_size() {
        assert_eq!(core::mem::size_of::<TxDescriptor>(), 16);
        assert_eq!(TxDescriptor::SIZE, core::mem::size_of::<TxDescriptor>());
    }

    #[test]
    fn tx_descriptor_alignment() {
        assert_eq!(core::mem::align_of::<TxDescriptor>(), 4);
    }

    // =========================================================================
    // TX Descriptor Ownership Tests
    // =========================================================================

    #[test]
    fn tx_descriptor_new_not_owned() {
        let desc = TxDescriptor::new();
        assert!(!desc.is_owned());
    }

    #[test]
    fn tx_descriptor_is_owned() {
        let desc = TxDescriptor::new();
        desc.set_owned();
        assert!(desc.is_owned());
        desc.clear_owned();
        assert!(!desc.is_owned());
    }

    #[test]
    fn tdes0_own_bit() {
        // OWN bit must be bit 31.
        let desc = TxDescriptor::new();
        desc.set_owned();
        assert_eq!(desc.raw_tdes0() & tdes0::OWN, tdes0::OWN);
        assert_eq!(tdes0::OWN, 1 << 31);
    }

    // =========================================================================
    // TX Descriptor Setup / Prepare Tests
    // =========================================================================

    #[test]
    fn tx_descriptor_setup_chained() {
        let desc = TxDescriptor::new();
        let buf = [0u8; 64];
        let next = TxDescriptor::new();

        desc.setup_chained(buf.as_ptr(), &next as *const TxDescriptor);

        assert_eq!(desc.buffer_addr(), buf.as_ptr() as u32);
        assert_eq!(desc.next_desc_addr(), &next as *const TxDescriptor as u32);
        assert!(desc.raw_tdes0() & tdes0::SECOND_ADDR_CHAINED != 0);
        assert!(!desc.is_owned());
    }

    #[test]
    fn tx_descriptor_prepare_single_frame() {
        let desc = TxDescriptor::new();
        desc.prepare(1500, true, true);

        let raw0 = desc.raw_tdes0();
        assert!(raw0 & tdes0::FIRST_SEGMENT != 0);
        assert!(raw0 & tdes0::LAST_SEGMENT != 0);
        assert!(raw0 & tdes0::INTERRUPT_ON_COMPLETE != 0);
        assert!(raw0 & tdes0::OWN == 0, "prepare must not set OWN");

        let len = desc.raw_tdes1() & tdes1::BUFFER1_SIZE_MASK;
        assert_eq!(len, 1500);
    }

    #[test]
    fn tdes0_first_last_bits() {
        let desc = TxDescriptor::new();

        // First segment only.
        desc.prepare(100, true, false);
        let raw = desc.raw_tdes0();
        assert!(raw & tdes0::FIRST_SEGMENT != 0);
        assert!(raw & tdes0::LAST_SEGMENT == 0);

        // Last segment only.
        desc.prepare(100, false, true);
        let raw = desc.raw_tdes0();
        assert!(raw & tdes0::FIRST_SEGMENT == 0);
        assert!(raw & tdes0::LAST_SEGMENT != 0);
    }

    #[test]
    fn tx_descriptor_prepare_and_submit() {
        let desc = TxDescriptor::new();
        desc.prepare_and_submit(256, true, true);
        assert!(desc.is_owned());
        assert_eq!(desc.raw_tdes1() & tdes1::BUFFER1_SIZE_MASK, 256);
    }

    #[test]
    fn tx_descriptor_no_errors_initially() {
        let desc = TxDescriptor::new();
        assert!(!desc.has_error());
        assert_eq!(desc.error_flags(), 0);
    }

    #[test]
    fn tx_descriptor_error_detection() {
        let desc = TxDescriptor::new();
        desc.tdes0.set(tdes0::ERR_SUMMARY | tdes0::UNDERFLOW_ERR);
        assert!(desc.has_error());
        assert!(desc.error_flags() & tdes0::UNDERFLOW_ERR != 0);
    }

    #[test]
    fn tx_descriptor_reset_preserves_chain() {
        let desc = TxDescriptor::new();
        let next_addr = 0x1234_5678u32;
        desc.next_desc_addr.set(next_addr);
        desc.prepare_and_submit(1000, true, true);

        desc.reset();

        assert!(!desc.is_owned());
        assert_eq!(desc.raw_tdes1() & tdes1::BUFFER1_SIZE_MASK, 0);
        assert_eq!(desc.next_desc_addr(), next_addr);
        assert!(desc.raw_tdes0() & tdes0::SECOND_ADDR_CHAINED != 0);
    }

    // =========================================================================
    // RX Descriptor Layout Tests
    // =========================================================================

    #[test]
    fn rx_descriptor_size() {
        assert_eq!(core::mem::size_of::<RxDescriptor>(), 16);
        assert_eq!(RxDescriptor::SIZE, core::mem::size_of::<RxDescriptor>());
    }

    #[test]
    fn rx_descriptor_alignment() {
        assert_eq!(core::mem::align_of::<RxDescriptor>(), 4);
    }

    // =========================================================================
    // RX Descriptor Ownership Tests
    // =========================================================================

    #[test]
    fn rx_descriptor_new_not_owned() {
        let desc = RxDescriptor::new();
        assert!(!desc.is_owned());
    }

    #[test]
    fn rdes0_own_bit() {
        let desc = RxDescriptor::new();
        desc.set_owned();
        assert_eq!(desc.raw_rdes0() & rdes0::OWN, rdes0::OWN);
        assert_eq!(rdes0::OWN, 1 << 31);
    }

    // =========================================================================
    // RX Descriptor Setup / Chained Tests
    // =========================================================================

    #[test]
    fn rx_descriptor_setup_chained() {
        let desc = RxDescriptor::new();
        let mut buf = [0u8; 1600];
        let next = RxDescriptor::new();

        desc.setup_chained(buf.as_mut_ptr(), 1600, &next as *const RxDescriptor);

        assert_eq!(desc.buffer_addr(), buf.as_ptr() as u32);
        assert_eq!(desc.next_desc_addr(), &next as *const RxDescriptor as u32);
        assert_eq!(desc.buffer_size(), 1600);
        assert!(desc.is_owned(), "setup_chained gives to DMA");
        assert!(desc.raw_rdes1() & rdes1::SECOND_ADDR_CHAINED != 0);
    }

    // =========================================================================
    // RX Descriptor Status Tests
    // =========================================================================

    #[test]
    fn rx_descriptor_first_last_flags() {
        let desc = RxDescriptor::new();
        assert!(!desc.is_first());
        assert!(!desc.is_last());

        desc.rdes0.set(rdes0::FIRST_DESC | rdes0::LAST_DESC);
        assert!(desc.is_first());
        assert!(desc.is_last());
        assert!(desc.is_complete_frame());
    }

    #[test]
    fn rx_descriptor_payload_length() {
        let desc = RxDescriptor::new();

        // Frame length 1504 (including CRC), payload = 1500.
        desc.rdes0.set(1504 << rdes0::FRAME_LEN_SHIFT);
        assert_eq!(desc.frame_length(), 1504);
        assert_eq!(desc.payload_length(), 1500);
    }

    #[test]
    fn rx_descriptor_payload_length_short_frame() {
        let desc = RxDescriptor::new();
        // Frame shorter than CRC — saturating_sub prevents underflow.
        desc.rdes0.set(2 << rdes0::FRAME_LEN_SHIFT);
        assert_eq!(desc.payload_length(), 0);
    }

    #[test]
    fn rx_descriptor_error_detection() {
        let desc = RxDescriptor::new();
        assert!(!desc.has_error());

        desc.rdes0
            .set(rdes0::ERR_SUMMARY | rdes0::CRC_ERR | rdes0::OVERFLOW_ERR);
        assert!(desc.has_error());
        assert!(desc.error_flags() & rdes0::CRC_ERR != 0);
        assert!(desc.error_flags() & rdes0::OVERFLOW_ERR != 0);
    }

    // =========================================================================
    // RX Descriptor Recycle Test
    // =========================================================================

    #[test]
    fn rx_descriptor_recycle() {
        let desc = RxDescriptor::new();
        desc.rdes1.set(1600);
        desc.rdes0
            .set(rdes0::FIRST_DESC | rdes0::LAST_DESC | (100 << rdes0::FRAME_LEN_SHIFT));

        desc.recycle();

        assert!(desc.is_owned());
        // Buffer size in RDES1 is preserved.
        assert_eq!(desc.buffer_size(), 1600);
    }
}
