// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! DMA engine managing TX/RX descriptor rings and frame I/O.
//!
//! The engine owns the descriptor rings and data buffers, providing
//! a high-level interface for frame transmission and reception without
//! any register access. Register programming is handled by the caller.

use crate::dma::descriptor::{RxDescriptor, TxDescriptor};
use crate::dma::ring::DescriptorRing;
use crate::error::EmacError;

/// DMA engine with statically allocated buffers.
///
/// # Const generics
/// - `RX`: number of RX descriptors/buffers
/// - `TX`: number of TX descriptors/buffers
/// - `BUF`: buffer size per descriptor (bytes)
pub struct DmaEngine<const RX: usize, const TX: usize, const BUF: usize> {
    /// RX descriptor ring.
    rx_ring: DescriptorRing<RxDescriptor, RX>,
    /// TX descriptor ring.
    tx_ring: DescriptorRing<TxDescriptor, TX>,
    /// RX data buffers.
    rx_buffers: [[u8; BUF]; RX],
    /// TX data buffers.
    tx_buffers: [[u8; BUF]; TX],
    /// Whether the engine has been initialized.
    initialized: bool,
}

impl<const RX: usize, const TX: usize, const BUF: usize> DmaEngine<RX, TX, BUF> {
    /// Create a new DMA engine (all zeroed, not yet initialized).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            rx_ring: DescriptorRing::new([const { RxDescriptor::new() }; RX]),
            tx_ring: DescriptorRing::new([const { TxDescriptor::new() }; TX]),
            rx_buffers: [[0u8; BUF]; RX],
            tx_buffers: [[0u8; BUF]; TX],
            initialized: false,
        }
    }

    /// Initialize descriptor chains.
    ///
    /// Sets up chained descriptors: each points to its buffer and the next
    /// descriptor. The last descriptor chains back to the first (circular).
    ///
    /// Returns `(rx_base_addr, tx_base_addr)` for programming DMA registers.
    pub fn init(&mut self) -> (u32, u32) {
        // Set up RX descriptors: each points to its buffer and next descriptor.
        for i in 0..RX {
            let next_idx = (i + 1) % RX;
            let buffer_ptr = self.rx_buffers[i].as_mut_ptr();
            let next_desc = self.rx_ring.get(next_idx) as *const RxDescriptor;
            self.rx_ring
                .get(i)
                .setup_chained(buffer_ptr, BUF, next_desc);
        }

        // Set up TX descriptors: each points to its buffer and next descriptor.
        for i in 0..TX {
            let next_idx = (i + 1) % TX;
            let buffer_ptr = self.tx_buffers[i].as_ptr();
            let next_desc = self.tx_ring.get(next_idx) as *const TxDescriptor;
            self.tx_ring.get(i).setup_chained(buffer_ptr, next_desc);
        }

        self.rx_ring.reset();
        self.tx_ring.reset();
        self.initialized = true;

        let rx_base = self.rx_ring.base_addr() as u32;
        let tx_base = self.tx_ring.base_addr() as u32;
        (rx_base, tx_base)
    }

    /// Check if initialized.
    #[inline(always)]
    #[must_use]
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Calculate total static memory usage in bytes.
    #[must_use]
    pub const fn memory_usage() -> usize {
        let rx_desc = RX * RxDescriptor::SIZE;
        let tx_desc = TX * TxDescriptor::SIZE;
        let rx_buf = RX * BUF;
        let tx_buf = TX * BUF;
        rx_desc + tx_desc + rx_buf + tx_buf
    }

    // ── Transmission ──────────────────────────────────────

    /// Check if there's room to transmit a frame of given length.
    #[must_use]
    pub fn can_transmit(&self, len: usize) -> bool {
        if len == 0 || len > BUF * TX {
            return false;
        }
        let needed = len.div_ceil(BUF);
        self.tx_available() >= needed
    }

    /// Count available (CPU-owned) TX descriptors starting from current.
    #[must_use]
    pub fn tx_available(&self) -> usize {
        let mut count = 0;
        for i in 0..TX {
            let idx = (self.tx_ring.current_index() + i) % TX;
            if !self.tx_ring.get(idx).is_owned() {
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    /// Submit a frame for transmission. Returns number of bytes sent.
    ///
    /// For frames larger than `BUF`, uses multiple descriptors (scatter-gather).
    /// Descriptors are given to DMA in reverse order to prevent a race where
    /// the DMA starts processing before all descriptors are ready.
    pub fn transmit(&mut self, data: &[u8]) -> Result<usize, EmacError> {
        if data.is_empty() {
            return Err(EmacError::InvalidLength);
        }

        if data.len() > BUF * TX {
            return Err(EmacError::FrameTooLarge);
        }

        let desc_count = data.len().div_ceil(BUF);
        if self.tx_available() < desc_count {
            return Err(EmacError::NoDescriptorsAvailable);
        }

        let current = self.tx_ring.current_index();
        let mut remaining = data.len();
        let mut offset = 0usize;

        // Prepare each descriptor with its data chunk.
        for i in 0..desc_count {
            let idx = (current + i) % TX;
            let desc = self.tx_ring.get(idx);

            if desc.is_owned() {
                return Err(EmacError::DescriptorBusy);
            }

            let chunk_size = core::cmp::min(remaining, BUF);
            self.tx_buffers[idx][..chunk_size].copy_from_slice(&data[offset..offset + chunk_size]);
            desc.prepare(chunk_size, i == 0, i == desc_count - 1);

            remaining -= chunk_size;
            offset += chunk_size;
        }

        // Give to DMA in reverse order (prevents race condition).
        for i in (0..desc_count).rev() {
            let idx = (current + i) % TX;
            self.tx_ring.get(idx).set_owned();
        }

        self.tx_ring.advance_by(desc_count);
        Ok(data.len())
    }

    /// Reclaim completed TX descriptors (return from DMA to CPU ownership).
    ///
    /// Returns the number of descriptors reclaimed.
    pub fn tx_reclaim(&mut self) -> usize {
        let mut reclaimed = 0;
        for i in 0..TX {
            let idx = (self.tx_ring.current_index() + i) % TX;
            let desc = self.tx_ring.get(idx);
            if !desc.is_owned() {
                reclaimed += 1;
            }
        }
        reclaimed
    }

    // ── Reception ─────────────────────────────────────────

    /// Check if a complete received frame is available — including
    /// multi-descriptor frames where the current descriptor is not
    /// itself the last one. Equivalent to
    /// `peek_frame_length().is_some()`.
    ///
    /// The previous implementation returned `true` only when the
    /// current descriptor carried the `LAST` bit, which silently
    /// disagreed with `receive` (which already supports multi-
    /// descriptor frames) on rings with `BUF < MTU`: the embassy-net
    /// driver would never wake on a scattered frame even though
    /// `receive` would happily reassemble it.
    #[must_use]
    pub fn rx_available(&self) -> bool {
        self.peek_frame_length().is_some()
    }

    /// Receive a frame into the provided buffer.
    ///
    /// Returns the number of bytes received, or `None` if no frame is ready.
    /// For single-descriptor frames, copies payload from the RX buffer.
    /// For multi-descriptor frames, copies from each descriptor's buffer.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<Option<usize>, EmacError> {
        let first_desc = self.rx_ring.current();

        // Not owned by CPU — no frame ready.
        if first_desc.is_owned() {
            return Ok(None);
        }

        // Single-descriptor frame (common case).
        if first_desc.is_first() && first_desc.is_last() {
            if first_desc.has_error() {
                first_desc.recycle();
                self.rx_ring.advance();
                return Err(EmacError::FrameError);
            }

            let frame_len = first_desc.payload_length();
            if buffer.len() < frame_len {
                first_desc.recycle();
                self.rx_ring.advance();
                return Err(EmacError::BufferTooSmall);
            }

            let idx = self.rx_ring.current_index();
            buffer[..frame_len].copy_from_slice(&self.rx_buffers[idx][..frame_len]);
            first_desc.recycle();
            self.rx_ring.advance();
            return Ok(Some(frame_len));
        }

        // Multi-descriptor frame: must start with first segment.
        if !first_desc.is_first() {
            self.flush_rx_frame();
            return Ok(None);
        }

        if first_desc.has_error() {
            self.flush_rx_frame();
            return Err(EmacError::FrameError);
        }

        // Walk descriptors to find total frame length.
        let mut frame_len = 0usize;
        let mut desc_count = 0usize;
        let current = self.rx_ring.current_index();

        for i in 0..RX {
            let idx = (current + i) % RX;
            let desc = self.rx_ring.get(idx);

            if desc.is_owned() {
                // Frame not complete yet.
                return Ok(None);
            }

            desc_count += 1;

            if desc.is_last() {
                frame_len = desc.payload_length();
                break;
            }
        }

        if frame_len == 0 {
            return Ok(None);
        }

        if buffer.len() < frame_len {
            self.flush_rx_frame();
            return Err(EmacError::BufferTooSmall);
        }

        // Copy data from all descriptors.
        let mut copied = 0usize;
        let last_desc_i = desc_count - 1;

        for i in 0..desc_count {
            let idx = (current + i) % RX;
            let copy_len = if i == last_desc_i {
                frame_len - copied
            } else {
                BUF
            };
            let copy_len = core::cmp::min(copy_len, frame_len - copied);

            if copy_len > 0 {
                buffer[copied..copied + copy_len]
                    .copy_from_slice(&self.rx_buffers[idx][..copy_len]);
                copied += copy_len;
            }
            self.rx_ring.get(idx).recycle();
        }

        self.rx_ring.advance_by(desc_count);
        Ok(Some(frame_len))
    }

    /// Get the length of the next available frame without consuming it.
    #[must_use]
    pub fn peek_frame_length(&self) -> Option<usize> {
        let desc = self.rx_ring.current();

        if desc.is_owned() {
            return None;
        }

        if desc.has_error() {
            return None;
        }

        // Complete single-descriptor frame.
        if desc.is_first() && desc.is_last() {
            return Some(desc.payload_length());
        }

        // Multi-descriptor: walk to find the last descriptor.
        if desc.is_first() {
            for i in 1..RX {
                let idx = (self.rx_ring.current_index() + i) % RX;
                let d = self.rx_ring.get(idx);

                if d.is_owned() {
                    return None;
                }

                if d.is_last() {
                    return Some(d.payload_length());
                }
            }
        }

        None
    }

    /// Count free RX descriptors (owned by DMA, ready to receive).
    #[must_use]
    pub fn rx_free_count(&self) -> usize {
        let mut count = 0;
        for i in 0..RX {
            if self.rx_ring.get(i).is_owned() {
                count += 1;
            }
        }
        count
    }

    /// Reset all descriptors to initial state.
    ///
    /// Re-initializes the chains and returns base addresses.
    pub fn reset(&mut self) -> (u32, u32) {
        self.init()
    }

    /// Discard the current RX frame (for errors or incomplete frames).
    fn flush_rx_frame(&mut self) {
        loop {
            let desc = self.rx_ring.current();

            if desc.is_owned() {
                break;
            }

            let is_last = desc.is_last();
            desc.recycle();
            self.rx_ring.advance();

            if is_last {
                break;
            }
        }
    }

    /// RX ring base address (for debugging).
    #[must_use]
    pub fn rx_ring_base(&self) -> u32 {
        self.rx_ring.base_addr() as u32
    }

    /// TX ring base address (for debugging).
    #[must_use]
    pub fn tx_ring_base(&self) -> u32 {
        self.tx_ring.base_addr() as u32
    }

    /// Current RX ring index.
    #[must_use]
    pub fn rx_current_index(&self) -> usize {
        self.rx_ring.current_index()
    }

    /// Current TX ring index.
    #[must_use]
    pub fn tx_current_index(&self) -> usize {
        self.tx_ring.current_index()
    }
}

impl<const RX: usize, const TX: usize, const BUF: usize> Default for DmaEngine<RX, TX, BUF> {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: DmaEngine can be shared between threads when properly synchronized.
unsafe impl<const RX: usize, const TX: usize, const BUF: usize> Sync for DmaEngine<RX, TX, BUF> {}

// SAFETY: DmaEngine can be sent between threads.
unsafe impl<const RX: usize, const TX: usize, const BUF: usize> Send for DmaEngine<RX, TX, BUF> {}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma::descriptor::bits::rdes0;

    // ── Initialization ────────────────────────────────────

    #[test]
    fn new_not_initialized() {
        let engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        assert!(!engine.is_initialized());
    }

    #[test]
    fn init_sets_initialized() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();
        assert!(engine.is_initialized());
    }

    #[test]
    fn init_returns_base_addresses() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let (rx_base, tx_base) = engine.init();

        // Both addresses must be non-zero.
        assert_ne!(rx_base, 0);
        assert_ne!(tx_base, 0);

        // RX and TX rings must have different addresses.
        assert_ne!(rx_base, tx_base);
    }

    #[test]
    fn init_chains_rx_descriptors() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let (rx_base, _) = engine.init();

        // After init, all RX descriptors are owned by DMA (setup_chained sets OWN).
        for i in 0..4 {
            let desc = engine.rx_ring.get(i);
            assert!(desc.is_owned(), "RX desc {} should be DMA-owned", i);
            assert_ne!(desc.buffer_addr(), 0, "RX desc {} buffer must be set", i);
            assert_ne!(desc.next_desc_addr(), 0, "RX desc {} chain must be set", i);
        }

        // Last descriptor chains back to first (circular).
        assert_eq!(engine.rx_ring.get(3).next_desc_addr(), rx_base);
    }

    #[test]
    fn init_chains_tx_descriptors() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let (_, tx_base) = engine.init();

        // After init, all TX descriptors are CPU-owned (setup_chained does not set OWN).
        for i in 0..4 {
            let desc = engine.tx_ring.get(i);
            assert!(!desc.is_owned(), "TX desc {} should be CPU-owned", i);
            assert_ne!(desc.buffer_addr(), 0, "TX desc {} buffer must be set", i);
            assert_ne!(desc.next_desc_addr(), 0, "TX desc {} chain must be set", i);
        }

        // Last descriptor chains back to first (circular).
        assert_eq!(engine.tx_ring.get(3).next_desc_addr(), tx_base);
    }

    // ── Memory usage ──────────────────────────────────────

    #[test]
    fn memory_usage_calculation() {
        // 4 * 32 (rx desc) + 4 * 32 (tx desc) + 4 * 256 (rx buf) + 4 * 256 (tx buf)
        // = 128 + 128 + 1024 + 1024 = 2304
        let usage = DmaEngine::<4, 4, 256>::memory_usage();
        assert_eq!(usage, 2304);
    }

    #[test]
    fn memory_usage_scales() {
        let small = DmaEngine::<2, 2, 512>::memory_usage();
        let large = DmaEngine::<10, 10, 1600>::memory_usage();
        assert!(large > small);
    }

    // ── TX available / can_transmit ───────────────────────

    #[test]
    fn tx_available_after_init() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();
        assert_eq!(engine.tx_available(), 4);
    }

    #[test]
    fn can_transmit_empty() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();
        assert!(!engine.can_transmit(0));
    }

    #[test]
    fn can_transmit_single_buffer() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();
        assert!(engine.can_transmit(100));
        assert!(engine.can_transmit(256));
    }

    #[test]
    fn can_transmit_too_large() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();
        // 4 * 256 = 1024, anything above should fail.
        assert!(!engine.can_transmit(1025));
    }

    #[test]
    fn can_transmit_multi_buffer() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();
        // 512 bytes needs 2 descriptors of 256 each.
        assert!(engine.can_transmit(512));
        // 1024 bytes needs all 4 descriptors.
        assert!(engine.can_transmit(1024));
    }

    // ── Transmit ──────────────────────────────────────────

    #[test]
    fn transmit_single_frame() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        let data = [0xABu8; 100];
        let result = engine.transmit(&data);
        assert_eq!(result, Ok(100));
    }

    #[test]
    fn transmit_sets_own_bit() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        let data = [0xABu8; 100];
        let _ = engine.transmit(&data);

        // After transmit, descriptor 0 should be DMA-owned.
        assert!(engine.tx_ring.get(0).is_owned());
        // TX ring should have advanced to index 1.
        assert_eq!(engine.tx_current_index(), 1);
    }

    #[test]
    fn transmit_copies_data_to_buffer() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        let data = [0xCA; 64];
        let _ = engine.transmit(&data);

        // Verify the data was copied to the TX buffer.
        assert_eq!(&engine.tx_buffers[0][..64], &data[..]);
    }

    #[test]
    fn transmit_scatter_gather() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        // 400 bytes needs 2 descriptors (256 + 144).
        let data = [0xBBu8; 400];
        let result = engine.transmit(&data);
        assert_eq!(result, Ok(400));

        // Both descriptors should be DMA-owned.
        assert!(engine.tx_ring.get(0).is_owned());
        assert!(engine.tx_ring.get(1).is_owned());

        // Ring should have advanced by 2.
        assert_eq!(engine.tx_current_index(), 2);
    }

    #[test]
    fn transmit_when_full() {
        let mut engine: DmaEngine<2, 2, 256> = DmaEngine::new();
        let _ = engine.init();

        // Fill both TX slots.
        let data = [0xAAu8; 100];
        assert!(engine.transmit(&data).is_ok());
        assert!(engine.transmit(&data).is_ok());

        // Third transmit should fail.
        let result = engine.transmit(&data);
        assert_eq!(result, Err(EmacError::NoDescriptorsAvailable));
    }

    #[test]
    fn transmit_empty_data_returns_error() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        let result = engine.transmit(&[]);
        assert_eq!(result, Err(EmacError::InvalidLength));
    }

    #[test]
    fn transmit_too_large_returns_error() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        let data = [0u8; 2048]; // 4 * 256 = 1024, so 2048 is too large.
        let result = engine.transmit(&data);
        assert_eq!(result, Err(EmacError::FrameTooLarge));
    }

    // ── TX reclaim ────────────────────────────────────────

    #[test]
    fn tx_reclaim_returns_completed() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        // Transmit a frame (gives descriptor 0 to DMA).
        let _ = engine.transmit(&[0xAA; 100]);
        assert!(engine.tx_ring.get(0).is_owned());

        // Simulate DMA completion by clearing OWN bit.
        engine.tx_ring.get(0).clear_owned();

        let reclaimed = engine.tx_reclaim();
        // All 4 descriptors are CPU-owned now (desc 0 cleared + 1,2,3 never submitted).
        assert_eq!(reclaimed, 4);
    }

    // ── RX available ──────────────────────────────────────

    #[test]
    fn rx_available_no_frame() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        // After init, all RX descriptors are DMA-owned.
        assert!(!engine.rx_available());
    }

    #[test]
    fn receive_no_frame_returns_none() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        let mut buf = [0u8; 256];
        let result = engine.receive(&mut buf);
        assert_eq!(result, Ok(None));
    }

    // ── Simulated RX ──────────────────────────────────────

    /// Helper: simulate a received single-descriptor frame.
    ///
    /// Writes data into the RX buffer at `desc_index`, then sets the
    /// descriptor's RDES0 to indicate a complete frame (first+last, frame
    /// length, CPU-owned).
    fn simulate_rx_frame(engine: &mut DmaEngine<4, 4, 256>, desc_index: usize, data: &[u8]) {
        // Copy payload into the RX buffer.
        engine.rx_buffers[desc_index][..data.len()].copy_from_slice(data);

        // Frame length in RDES0 includes the 4-byte CRC.
        let frame_len_with_crc = (data.len() + 4) as u32;
        let rdes0_val =
            rdes0::FIRST_DESC | rdes0::LAST_DESC | (frame_len_with_crc << rdes0::FRAME_LEN_SHIFT);
        // OWN bit is NOT set — CPU owns it (simulates DMA completion).
        engine.rx_ring.get(desc_index).set_raw_rdes0(rdes0_val);
    }

    #[test]
    fn receive_simulated_frame() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        // Simulate receiving a 64-byte frame at descriptor 0.
        let payload = [0xDE; 64];
        simulate_rx_frame(&mut engine, 0, &payload);

        let mut buf = [0u8; 256];
        let result = engine.receive(&mut buf);
        assert_eq!(result, Ok(Some(64)));
        assert_eq!(&buf[..64], &payload[..]);
    }

    #[test]
    fn receive_advances_ring() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        simulate_rx_frame(&mut engine, 0, &[0xAA; 32]);
        assert_eq!(engine.rx_current_index(), 0);

        let mut buf = [0u8; 256];
        let _ = engine.receive(&mut buf);
        assert_eq!(engine.rx_current_index(), 1);
    }

    #[test]
    fn receive_recycles_descriptor() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        simulate_rx_frame(&mut engine, 0, &[0xAA; 32]);
        let mut buf = [0u8; 256];
        let _ = engine.receive(&mut buf);

        // After receive, the descriptor should be recycled (DMA-owned again).
        assert!(engine.rx_ring.get(0).is_owned());
    }

    #[test]
    fn peek_frame_length_returns_size() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        simulate_rx_frame(&mut engine, 0, &[0xBB; 100]);

        let len = engine.peek_frame_length();
        assert_eq!(len, Some(100));

        // peek should NOT consume the frame.
        assert!(engine.rx_available());
    }

    #[test]
    fn peek_frame_length_none_when_empty() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        assert_eq!(engine.peek_frame_length(), None);
    }

    /// Simulate a two-descriptor RX frame: `desc[start]` carries
    /// `FIRST` + payload[..mid], `desc[start+1]` carries `LAST` +
    /// payload[mid..] with the total frame length in its RDES0.
    fn simulate_rx_frame_two_descriptors(
        engine: &mut DmaEngine<4, 4, 256>,
        start: usize,
        chunks: (&[u8], &[u8]),
    ) {
        let (a, b) = chunks;
        engine.rx_buffers[start][..a.len()].copy_from_slice(a);
        engine.rx_buffers[start + 1][..b.len()].copy_from_slice(b);

        // First descriptor: FIRST, no LAST, no length encoded
        // (length lives in the LAST descriptor on this Synopsys core).
        engine.rx_ring.get(start).set_raw_rdes0(rdes0::FIRST_DESC);

        // Last descriptor: LAST, no FIRST, total frame length (incl. CRC).
        let frame_len_with_crc = (a.len() + b.len() + 4) as u32;
        let rdes0_val = rdes0::LAST_DESC | (frame_len_with_crc << rdes0::FRAME_LEN_SHIFT);
        engine.rx_ring.get(start + 1).set_raw_rdes0(rdes0_val);
    }

    #[test]
    fn rx_available_true_for_multi_descriptor_frame() {
        // Regression: previous `rx_available` returned `!current.is_owned() &&
        // current.is_last()`, which silently said "no frame" for any RX
        // chain where the current descriptor was the FIRST (not LAST)
        // chunk. Now it must agree with `peek_frame_length`.
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        let chunk_a = [0xAA; 200];
        let chunk_b = [0xBB; 100];
        simulate_rx_frame_two_descriptors(&mut engine, 0, (&chunk_a, &chunk_b));

        // `current` points at desc[0] which is FIRST but NOT LAST. The
        // pre-fix implementation returned `false` here.
        assert!(
            engine.rx_available(),
            "rx_available must report a multi-descriptor frame as ready"
        );
        assert_eq!(engine.peek_frame_length(), Some(300));
    }

    #[test]
    fn rx_free_count_after_init() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        // After init, all RX descriptors are DMA-owned (free for receiving).
        assert_eq!(engine.rx_free_count(), 4);
    }

    #[test]
    fn rx_free_count_after_receive() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        simulate_rx_frame(&mut engine, 0, &[0xCC; 48]);
        // One descriptor is now CPU-owned (received but not yet processed).
        assert_eq!(engine.rx_free_count(), 3);

        // Process it.
        let mut buf = [0u8; 256];
        let _ = engine.receive(&mut buf);

        // After receive, it's recycled back to DMA.
        assert_eq!(engine.rx_free_count(), 4);
    }

    #[test]
    fn receive_buffer_too_small() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        simulate_rx_frame(&mut engine, 0, &[0xDD; 200]);

        let mut buf = [0u8; 32]; // Too small for 200 bytes.
        let result = engine.receive(&mut buf);
        assert_eq!(result, Err(EmacError::BufferTooSmall));

        // Descriptor should still be recycled after error.
        assert!(engine.rx_ring.get(0).is_owned());
    }

    // ── Reset ─────────────────────────────────────────────

    #[test]
    fn reset_reinitializes() {
        let mut engine: DmaEngine<4, 4, 256> = DmaEngine::new();
        let _ = engine.init();

        // Transmit something to change state.
        let _ = engine.transmit(&[0xAA; 100]);
        assert_eq!(engine.tx_current_index(), 1);

        // Reset.
        let (rx_base, tx_base) = engine.reset();
        assert_ne!(rx_base, 0);
        assert_ne!(tx_base, 0);

        // Ring indices should be back to 0.
        assert_eq!(engine.rx_current_index(), 0);
        assert_eq!(engine.tx_current_index(), 0);

        // All TX descriptors should be CPU-owned.
        assert_eq!(engine.tx_available(), 4);

        // All RX descriptors should be DMA-owned.
        assert_eq!(engine.rx_free_count(), 4);
    }

    // ── Default trait ─────────────────────────────────────

    #[test]
    fn default_trait() {
        let d1: DmaEngine<4, 4, 256> = DmaEngine::new();
        let d2: DmaEngine<4, 4, 256> = DmaEngine::default();
        assert_eq!(d1.is_initialized(), d2.is_initialized());
    }
}
