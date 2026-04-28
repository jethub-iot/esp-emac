// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Interrupt status parsing for the ESP32 EMAC DMA controller.
//!
//! [`InterruptStatus`] decodes the raw `DMASTATUS` register value into
//! the individual TX/RX/error flags an interrupt handler typically wants.

use crate::regs::dma::status;

/// Decoded DMA status flags.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptStatus {
    /// TX complete — a frame was transmitted successfully.
    pub tx_complete: bool,
    /// TX DMA stopped.
    pub tx_stopped: bool,
    /// No TX descriptors available.
    pub tx_buf_unavailable: bool,
    /// TX FIFO underflow.
    pub tx_underflow: bool,
    /// RX complete — a frame was received.
    pub rx_complete: bool,
    /// RX DMA stopped.
    pub rx_stopped: bool,
    /// No RX descriptors available (DMA suspended).
    pub rx_buf_unavailable: bool,
    /// RX FIFO overflow.
    pub rx_overflow: bool,
    /// Fatal bus error — unrecoverable DMA error.
    pub fatal_bus_error: bool,
    /// Normal interrupt summary.
    pub normal_summary: bool,
    /// Abnormal interrupt summary.
    pub abnormal_summary: bool,
}

impl InterruptStatus {
    /// Decode from a raw `DMASTATUS` register value.
    #[inline]
    #[must_use]
    pub fn from_raw(raw: u32) -> Self {
        Self {
            tx_complete: (raw & status::TI) != 0,
            tx_stopped: (raw & status::TPS) != 0,
            tx_buf_unavailable: (raw & status::TU) != 0,
            tx_underflow: (raw & status::UNF) != 0,
            rx_complete: (raw & status::RI) != 0,
            rx_stopped: (raw & status::RPS) != 0,
            rx_buf_unavailable: (raw & status::RU) != 0,
            rx_overflow: (raw & status::OVF) != 0,
            fatal_bus_error: (raw & status::FBI) != 0,
            normal_summary: (raw & status::NIS) != 0,
            abnormal_summary: (raw & status::AIS) != 0,
        }
    }

    /// Encode back to a raw value (for write-1-to-clear).
    #[inline]
    #[must_use]
    pub fn to_raw(&self) -> u32 {
        let mut v = 0u32;
        if self.tx_complete {
            v |= status::TI;
        }
        if self.tx_stopped {
            v |= status::TPS;
        }
        if self.tx_buf_unavailable {
            v |= status::TU;
        }
        if self.tx_underflow {
            v |= status::UNF;
        }
        if self.rx_complete {
            v |= status::RI;
        }
        if self.rx_stopped {
            v |= status::RPS;
        }
        if self.rx_buf_unavailable {
            v |= status::RU;
        }
        if self.rx_overflow {
            v |= status::OVF;
        }
        if self.fatal_bus_error {
            v |= status::FBI;
        }
        if self.normal_summary {
            v |= status::NIS;
        }
        if self.abnormal_summary {
            v |= status::AIS;
        }
        v
    }

    /// True if any non-summary bit is set.
    #[inline]
    #[must_use]
    pub fn any(&self) -> bool {
        self.tx_complete
            || self.tx_stopped
            || self.tx_buf_unavailable
            || self.tx_underflow
            || self.rx_complete
            || self.rx_stopped
            || self.rx_buf_unavailable
            || self.rx_overflow
            || self.fatal_bus_error
    }

    /// True if any error flag is set.
    #[inline]
    #[must_use]
    pub fn has_error(&self) -> bool {
        self.tx_underflow || self.rx_overflow || self.fatal_bus_error
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_raw_zero() {
        let s = InterruptStatus::from_raw(0);
        assert!(!s.any());
        assert!(!s.has_error());
    }

    #[test]
    fn tx_rx_complete() {
        let s = InterruptStatus::from_raw(status::TI | status::RI | status::NIS);
        assert!(s.tx_complete);
        assert!(s.rx_complete);
        assert!(s.normal_summary);
        assert!(s.any());
        assert!(!s.has_error());
    }

    #[test]
    fn errors() {
        let s = InterruptStatus::from_raw(status::FBI | status::OVF);
        assert!(s.fatal_bus_error);
        assert!(s.rx_overflow);
        assert!(s.has_error());
    }

    #[test]
    fn roundtrip() {
        let raw = status::TI | status::RI | status::NIS | status::AIS | status::FBI;
        let s = InterruptStatus::from_raw(raw);
        assert_eq!(s.to_raw(), raw);
    }

    #[test]
    fn any_excludes_summary_bits() {
        let s = InterruptStatus::from_raw(status::NIS | status::AIS);
        assert!(!s.any());
    }
}
