// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Generic circular ring buffer for DMA descriptors.
//!
//! The ring wraps a fixed-size array of descriptors and maintains a
//! current-index that advances with wraparound. This is the core
//! data structure used by the DMA engine to walk TX and RX rings.

/// Circular descriptor ring with wraparound index.
pub struct DescriptorRing<D, const N: usize> {
    /// Array of descriptors.
    descriptors: [D; N],
    /// Current processing index.
    current: usize,
}

impl<D, const N: usize> DescriptorRing<D, N> {
    /// Create a new descriptor ring from an existing array.
    #[must_use]
    pub const fn new(descriptors: [D; N]) -> Self {
        Self {
            descriptors,
            current: 0,
        }
    }

    /// Number of descriptors in the ring.
    #[inline(always)]
    #[must_use]
    pub const fn len(&self) -> usize {
        N
    }

    /// Check if the ring is empty (always false for non-zero `N`).
    #[inline(always)]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        N == 0
    }

    /// Current index.
    #[inline(always)]
    #[must_use]
    pub const fn current_index(&self) -> usize {
        self.current
    }

    /// Advance the current index by one, wrapping around.
    #[inline(always)]
    pub fn advance(&mut self) {
        self.current = (self.current + 1) % N;
    }

    /// Reset the current index to 0.
    #[inline(always)]
    pub fn reset(&mut self) {
        self.current = 0;
    }

    /// Reference to the current descriptor.
    #[inline(always)]
    pub fn current(&self) -> &D {
        &self.descriptors[self.current]
    }

    /// Mutable reference to the current descriptor.
    #[inline(always)]
    pub fn current_mut(&mut self) -> &mut D {
        &mut self.descriptors[self.current]
    }

    /// Reference to the descriptor at `index` (wraps around).
    #[inline(always)]
    pub fn get(&self, index: usize) -> &D {
        &self.descriptors[index % N]
    }

    /// Mutable reference to the descriptor at `index` (wraps around).
    #[inline(always)]
    pub fn get_mut(&mut self, index: usize) -> &mut D {
        &mut self.descriptors[index % N]
    }

    /// Pointer to the first descriptor (for programming DMA base-address registers).
    #[inline(always)]
    pub fn base_addr(&self) -> *const D {
        self.descriptors.as_ptr()
    }

    /// Iterate over all descriptors.
    pub fn iter(&self) -> impl Iterator<Item = &D> {
        self.descriptors.iter()
    }

    /// Iterate mutably over all descriptors.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut D> {
        self.descriptors.iter_mut()
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_len() {
        let ring = DescriptorRing::new([0u32; 8]);
        assert_eq!(ring.len(), 8);
        assert!(!ring.is_empty());
    }

    #[test]
    fn ring_advance_wraps() {
        let mut ring = DescriptorRing::new([0u32; 4]);
        assert_eq!(ring.current_index(), 0);
        ring.advance();
        assert_eq!(ring.current_index(), 1);
        ring.advance();
        assert_eq!(ring.current_index(), 2);
        ring.advance();
        assert_eq!(ring.current_index(), 3);
        ring.advance();
        assert_eq!(ring.current_index(), 0); // wrapped
    }

    #[test]
    fn ring_current_changes_after_advance() {
        let mut ring = DescriptorRing::new([10u32, 20, 30]);
        assert_eq!(*ring.current(), 10);
        ring.advance();
        assert_eq!(*ring.current(), 20);
        ring.advance();
        assert_eq!(*ring.current(), 30);
    }

    #[test]
    fn ring_reset() {
        let mut ring = DescriptorRing::new([0u32; 4]);
        ring.advance();
        ring.advance();
        assert_eq!(ring.current_index(), 2);
        ring.reset();
        assert_eq!(ring.current_index(), 0);
    }

    #[test]
    fn ring_base_addr() {
        let ring = DescriptorRing::new([10u32, 20, 30]);
        let ptr = ring.base_addr();
        assert!(!ptr.is_null());
        // SAFETY: `base_addr` points to the first element of a valid array.
        unsafe {
            assert_eq!(*ptr, 10);
        }
    }

    #[test]
    fn ring_get_by_index() {
        let ring = DescriptorRing::new([10u32, 20, 30, 40]);
        assert_eq!(*ring.get(0), 10);
        assert_eq!(*ring.get(1), 20);
        assert_eq!(*ring.get(3), 40);
    }

    #[test]
    fn ring_get_wraps() {
        let ring = DescriptorRing::new([10u32, 20, 30, 40]);
        assert_eq!(*ring.get(4), 10); // wraps
        assert_eq!(*ring.get(5), 20);
    }

    #[test]
    fn ring_get_mut_modifies() {
        let mut ring = DescriptorRing::new([10u32, 20, 30]);
        *ring.get_mut(1) = 999;
        assert_eq!(*ring.get(1), 999);
    }

    #[test]
    fn ring_current_mut_modifies() {
        let mut ring = DescriptorRing::new([10u32, 20, 30]);
        *ring.current_mut() = 42;
        assert_eq!(*ring.current(), 42);
    }

    #[test]
    fn ring_iter() {
        let ring = DescriptorRing::new([1u32, 2, 3, 4]);
        let mut iter = ring.iter();
        assert_eq!(iter.next(), Some(&1));
        assert_eq!(iter.next(), Some(&2));
        assert_eq!(iter.next(), Some(&3));
        assert_eq!(iter.next(), Some(&4));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn ring_iter_mut() {
        let mut ring = DescriptorRing::new([1u32, 2, 3, 4]);
        for val in ring.iter_mut() {
            *val *= 10;
        }
        assert_eq!(*ring.get(0), 10);
        assert_eq!(*ring.get(1), 20);
        assert_eq!(*ring.get(2), 30);
        assert_eq!(*ring.get(3), 40);
    }

    #[test]
    fn ring_single_element() {
        let mut ring = DescriptorRing::new([42u32]);
        assert_eq!(ring.len(), 1);
        assert_eq!(*ring.current(), 42);
        ring.advance();
        assert_eq!(ring.current_index(), 0); // wraps immediately
    }

    #[test]
    fn ring_wraparound_stress() {
        let mut ring = DescriptorRing::new([0u32; 7]);
        for i in 0..100 {
            assert_eq!(ring.current_index(), i % 7);
            ring.advance();
        }
    }
}
