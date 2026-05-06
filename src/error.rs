// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! EMAC error types.

/// EMAC driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmacError {
    /// MDIO operation timed out (no PHY response within the SMI window).
    Timeout,
    /// DMA software reset (`DMABUSMODE.SWR`) did not self-clear within
    /// the [`crate::reset::ResetController`] timeout. Distinct from
    /// [`Self::Timeout`] so callers can tell SMI-bus errors apart from
    /// DMA-controller stuckness.
    DmaResetTimeout,
    /// DMA descriptor or buffer error.
    DmaError,
    /// Invalid configuration (bad pin, clock, etc.).
    InvalidConfig,
    /// PHY address out of range (must be 0-31).
    InvalidPhyAddress,
    /// Frame length is zero or otherwise invalid.
    InvalidLength,
    /// Frame too large for available descriptors/buffers.
    FrameTooLarge,
    /// No TX descriptors available (ring full).
    NoDescriptorsAvailable,
    /// Descriptor is still owned by DMA.
    DescriptorBusy,
    /// No received frame available.
    NoFrameAvailable,
    /// Received frame has errors (CRC, overflow, etc.).
    FrameError,
    /// Caller-provided buffer too small for the received frame.
    BufferTooSmall,
    /// DMA engine not initialized.
    NotInitialized,
    /// `init()` was called twice.
    AlreadyInitialized,
    /// `stop()` could not flush the TX FIFO within
    /// `TX_FIFO_FLUSH_TIMEOUT_US`. Teardown still proceeded — the
    /// driver state is back to [`crate::EmacState::Initialized`] and
    /// MAC/DMA are quiesced — but at least one in-flight TX frame may
    /// have been truncated on the wire. The caller decides whether
    /// to treat this as a recoverable warning (sample, log,
    /// `start()` again — the engine is ready) or as terminal: there
    /// is no in-crate "re-init" path because [`crate::Emac::init`] is
    /// one-shot, so a hard recovery means a peripheral / SoC reset
    /// from the application layer.
    TxFlushTimeout,
}

impl From<crate::reset::ResetError> for EmacError {
    fn from(e: crate::reset::ResetError) -> Self {
        match e {
            crate::reset::ResetError::Timeout => EmacError::DmaResetTimeout,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate alloc;

    #[test]
    fn emac_error_debug() {
        let err = EmacError::Timeout;
        let dbg = alloc::format!("{:?}", err);
        assert!(dbg.contains("Timeout"));
    }

    #[test]
    fn emac_error_equality() {
        assert_eq!(EmacError::Timeout, EmacError::Timeout);
        assert_ne!(EmacError::Timeout, EmacError::DmaError);
        assert_ne!(EmacError::InvalidConfig, EmacError::InvalidPhyAddress);
    }

    #[test]
    fn emac_error_clone() {
        let err = EmacError::DmaError;
        let cloned = err;
        assert_eq!(err, cloned);
    }

    #[test]
    fn reset_timeout_maps_to_dma_reset_timeout_not_mdio_timeout() {
        // Regression: previously `From<ResetError>` collapsed DMA reset
        // timeouts into the MDIO-flavoured `EmacError::Timeout`,
        // making the two indistinguishable for API consumers. They
        // are distinct hardware failure modes — keep the conversion
        // routed at the dedicated variant.
        let mapped: EmacError = crate::reset::ResetError::Timeout.into();
        assert_eq!(mapped, EmacError::DmaResetTimeout);
        assert_ne!(mapped, EmacError::Timeout);
    }
}
