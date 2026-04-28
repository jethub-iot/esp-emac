// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! EMAC error types.

/// EMAC driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmacError {
    /// MDIO operation timed out.
    Timeout,
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
}

impl From<ph_esp32_mac::Error> for EmacError {
    fn from(e: ph_esp32_mac::Error) -> Self {
        use ph_esp32_mac::{ConfigError, DmaError, Error, IoError};
        match e {
            Error::Config(ConfigError::AlreadyInitialized)
            | Error::Config(ConfigError::InvalidConfig)
            | Error::Config(ConfigError::ClockError)
            | Error::Config(ConfigError::GpioError) => EmacError::InvalidConfig,
            Error::Config(ConfigError::InvalidPhyAddress) => EmacError::InvalidPhyAddress,
            Error::Config(ConfigError::ResetFailed) => EmacError::Timeout,
            Error::Dma(DmaError::NoDescriptorsAvailable) => EmacError::NoDescriptorsAvailable,
            Error::Dma(DmaError::DescriptorBusy) => EmacError::DescriptorBusy,
            Error::Dma(DmaError::FrameTooLarge) => EmacError::FrameTooLarge,
            Error::Dma(DmaError::InvalidLength) => EmacError::InvalidLength,
            Error::Dma(DmaError::FatalBusError) => EmacError::DmaError,
            Error::Io(IoError::Timeout) => EmacError::Timeout,
            Error::Io(IoError::InvalidState) => EmacError::NotInitialized,
            Error::Io(IoError::BufferTooSmall) => EmacError::BufferTooSmall,
            Error::Io(IoError::IncompleteFrame) => EmacError::NoFrameAvailable,
            Error::Io(IoError::FrameError) => EmacError::FrameError,
            Error::Io(IoError::PhyError) => EmacError::Timeout,
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
}
