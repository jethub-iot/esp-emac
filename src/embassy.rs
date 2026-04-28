// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Native embassy-net driver for the ESP32 EMAC.
//!
//! Wraps [`crate::Emac`] directly (no `ph_esp32_mac::EmbassyEmac` proxy).
//! [`EmacDriverState`] holds the wakers and link cache and is intended
//! to live in `static` storage so the ISR can wake stack tasks.
//!
//! # Usage
//!
//! ```ignore
//! static mut EMAC: Emac<10, 10, 1600> = Emac::new(EmacConfig::default());
//! static EMAC_STATE: EmacDriverState = EmacDriverState::new();
//!
//! // After bringing the EMAC up:
//! emac.bind_interrupt(emac_isr);
//! emac.start().unwrap();
//! let driver = EmacDriver::new(emac, &EMAC_STATE);
//! ```

use core::cell::Cell;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Context;

use critical_section::Mutex;
use embassy_net_driver::{
    Capabilities, ChecksumCapabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken,
};
use embassy_sync::waitqueue::AtomicWaker;

use crate::emac::Emac;
use crate::interrupt::InterruptStatus;

/// Diagnostic snapshot of the ISR counters.
#[derive(Debug, Clone, Copy, Default)]
pub struct IrqCounters {
    /// Total number of times the ISR ran.
    pub total: u32,
    /// `RI` (rx_complete) flag observed.
    pub ri: u32,
    /// `RU` (rx_buf_unavailable) flag observed.
    pub ru: u32,
    /// `TI` (tx_complete) flag observed.
    pub ti: u32,
    /// `TU` (tx_buf_unavailable) flag observed.
    pub tu: u32,
    /// `ERI` (early receive) flag observed.
    pub eri: u32,
    /// At least one error flag observed (UNF/OVF/FBI).
    pub err: u32,
    /// Last raw `DMASTATUS` snapshot taken in the ISR (before W1C).
    pub last_dmastat: u32,
}

/// Maximum frame size for stack-allocated copy buffers (Ethernet MTU + headers).
const MAX_FRAME_SIZE: usize = 1600;

/// Maximum transmission unit reported to embassy-net (IP MTU + Ethernet header).
const MTU: usize = 1514;

// =============================================================================
// Driver state
// =============================================================================

/// Shared state for the embassy-net driver.
///
/// Holds the RX, TX, and link wakers plus the cached link state. Place
/// in a `static` so it can be reached from the EMAC ISR.
pub struct EmacDriverState {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
    link_waker: AtomicWaker,
    link_state: Mutex<Cell<LinkState>>,
    /// Diagnostic counters — incremented in the ISR. Used by the dev-log
    /// hypotheses H6/H7.
    irq_count: AtomicU32,
    irq_ri: AtomicU32,
    irq_ru: AtomicU32,
    irq_ti: AtomicU32,
    irq_tu: AtomicU32,
    irq_eri: AtomicU32,
    irq_err: AtomicU32,
    /// Last observed raw DMASTAT (snapshot taken in ISR, before W1C).
    last_dmastat: AtomicU32,
    /// Counters bumped by [`EmacDriver::receive`] / [`EmacDriver::transmit`]
    /// to see how often embassy-net actually pulls data.
    drv_rx_calls: AtomicU32,
    drv_rx_some: AtomicU32,
    drv_tx_calls: AtomicU32,
    drv_tx_some: AtomicU32,
}

impl Default for EmacDriverState {
    fn default() -> Self {
        Self::new()
    }
}

impl EmacDriverState {
    /// Create a new state with link initially down.
    pub const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            link_waker: AtomicWaker::new(),
            link_state: Mutex::new(Cell::new(LinkState::Down)),
            irq_count: AtomicU32::new(0),
            irq_ri: AtomicU32::new(0),
            irq_ru: AtomicU32::new(0),
            irq_ti: AtomicU32::new(0),
            irq_tu: AtomicU32::new(0),
            irq_eri: AtomicU32::new(0),
            irq_err: AtomicU32::new(0),
            last_dmastat: AtomicU32::new(0),
            drv_rx_calls: AtomicU32::new(0),
            drv_rx_some: AtomicU32::new(0),
            drv_tx_calls: AtomicU32::new(0),
            drv_tx_some: AtomicU32::new(0),
        }
    }

    /// Diagnostic counters from the ISR.
    pub fn irq_counters(&self) -> IrqCounters {
        IrqCounters {
            total: self.irq_count.load(Ordering::Relaxed),
            ri: self.irq_ri.load(Ordering::Relaxed),
            ru: self.irq_ru.load(Ordering::Relaxed),
            ti: self.irq_ti.load(Ordering::Relaxed),
            tu: self.irq_tu.load(Ordering::Relaxed),
            eri: self.irq_eri.load(Ordering::Relaxed),
            err: self.irq_err.load(Ordering::Relaxed),
            last_dmastat: self.last_dmastat.load(Ordering::Relaxed),
        }
    }

    /// Diagnostic counters from `Driver::receive` / `Driver::transmit`.
    pub fn driver_counters(&self) -> (u32, u32, u32, u32) {
        (
            self.drv_rx_calls.load(Ordering::Relaxed),
            self.drv_rx_some.load(Ordering::Relaxed),
            self.drv_tx_calls.load(Ordering::Relaxed),
            self.drv_tx_some.load(Ordering::Relaxed),
        )
    }

    /// Read the cached link state.
    pub fn link_state(&self) -> LinkState {
        critical_section::with(|cs| self.link_state.borrow(cs).get())
    }

    /// Update the cached link state and wake stack tasks.
    pub fn set_link_state(&self, state: LinkState) {
        critical_section::with(|cs| self.link_state.borrow(cs).set(state));
        self.link_waker.wake();
    }

    /// Mark the link as up and wake the stack.
    pub fn set_link_up(&self) {
        self.set_link_state(LinkState::Up);
    }

    /// Mark the link as down and wake the stack.
    pub fn set_link_down(&self) {
        self.set_link_state(LinkState::Down);
    }

    /// Wake RX/TX tasks based on a snapshot of the DMA interrupt status.
    pub fn on_interrupt_status(&self, status: InterruptStatus) {
        if status.rx_complete || status.rx_buf_unavailable {
            self.rx_waker.wake();
        }
        if status.tx_complete || status.tx_buf_unavailable {
            self.tx_waker.wake();
        }
        if status.has_error() {
            self.rx_waker.wake();
            self.tx_waker.wake();
        }
    }

    /// Read the DMA status register, clear the interrupts, and wake any
    /// tasks waiting on RX/TX/link.
    ///
    /// Intended to be called from the EMAC ISR. Touches only memory-mapped
    /// EMAC registers and the embedded wakers, so there is no aliasing
    /// concern with the [`EmacDriver`] holding a raw pointer to the
    /// [`Emac`] state.
    pub fn handle_emac_interrupt(&self) {
        let dmastat = crate::regs::dma::BASE + crate::regs::dma::DMASTATUS;
        // SAFETY: DMASTATUS is a known-valid 32-bit memory-mapped register.
        let raw = unsafe { core::ptr::read_volatile(dmastat as *const u32) };
        let status = InterruptStatus::from_raw(raw);
        // Write-1-to-clear.
        // SAFETY: same address; bits are W1C — writing the snapshot back
        // clears every flag the snapshot observed.
        unsafe { core::ptr::write_volatile(dmastat as *mut u32, status.to_raw()) };

        self.irq_count.fetch_add(1, Ordering::Relaxed);
        self.last_dmastat.store(raw, Ordering::Relaxed);
        if status.rx_complete {
            self.irq_ri.fetch_add(1, Ordering::Relaxed);
        }
        if status.rx_buf_unavailable {
            self.irq_ru.fetch_add(1, Ordering::Relaxed);
        }
        if status.tx_complete {
            self.irq_ti.fetch_add(1, Ordering::Relaxed);
        }
        if status.tx_buf_unavailable {
            self.irq_tu.fetch_add(1, Ordering::Relaxed);
        }
        // Early Receive Interrupt (ETI) is bit 14. We don't expose it in
        // InterruptStatus today, so check the raw flag explicitly.
        if (raw & (1 << 14)) != 0 {
            self.irq_eri.fetch_add(1, Ordering::Relaxed);
        }
        if status.has_error() {
            self.irq_err.fetch_add(1, Ordering::Relaxed);
        }

        self.on_interrupt_status(status);
    }
}

// =============================================================================
// Driver wrapper
// =============================================================================

/// embassy-net driver for the ESP32 EMAC.
///
/// The driver holds a raw pointer to a previously-initialized
/// [`Emac`] together with a reference to a shared [`EmacDriverState`].
///
/// # Safety
///
/// The pointer is dereferenced in `Driver` impl methods. The lifetime
/// `'d` ensures the underlying `Emac` outlives the driver, but the raw
/// pointer means **mutable aliasing** would be unsound. Construct only
/// one driver per `Emac` instance and let it own access until shutdown.
pub struct EmacDriver<'d, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    state: &'d EmacDriverState,
    _marker: PhantomData<&'d mut Emac<RX, TX, BUF>>,
}

unsafe impl<const RX: usize, const TX: usize, const BUF: usize> Send
    for EmacDriver<'_, RX, TX, BUF>
{
}

impl<'d, const RX: usize, const TX: usize, const BUF: usize> EmacDriver<'d, RX, TX, BUF> {
    /// Create a new embassy-net driver.
    ///
    /// `emac` must be already initialized and started; `state` must be
    /// the same instance whose [`on_interrupt_status`] is called from
    /// the EMAC ISR.
    pub fn new(emac: &'d mut Emac<RX, TX, BUF>, state: &'d EmacDriverState) -> Self {
        Self {
            emac: emac as *mut _,
            state,
            _marker: PhantomData,
        }
    }

    /// Borrow the shared state.
    pub fn state(&self) -> &EmacDriverState {
        self.state
    }
}

// =============================================================================
// RX / TX tokens
// =============================================================================

/// embassy-net RX token — copies one received frame on `consume`.
pub struct EmacRxToken<'a, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    _marker: PhantomData<&'a mut Emac<RX, TX, BUF>>,
}

impl<const RX: usize, const TX: usize, const BUF: usize> RxToken for EmacRxToken<'_, RX, TX, BUF> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buffer = [0u8; MAX_FRAME_SIZE];
        // SAFETY: `EmacDriver` guarantees the pointer is valid for the
        // lifetime tracked by `'a`; tokens are consumed synchronously by
        // the embassy stack.
        let emac = unsafe { &mut *self.emac };
        let len = emac.receive(&mut buffer).ok().flatten().unwrap_or(0);
        f(&mut buffer[..len])
    }
}

/// embassy-net TX token — submits one frame on `consume`.
pub struct EmacTxToken<'a, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    _marker: PhantomData<&'a mut Emac<RX, TX, BUF>>,
}

impl<const RX: usize, const TX: usize, const BUF: usize> TxToken for EmacTxToken<'_, RX, TX, BUF> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let len = len.min(MAX_FRAME_SIZE);
        let mut buffer = [0u8; MAX_FRAME_SIZE];
        let result = f(&mut buffer[..len]);
        // SAFETY: see `EmacRxToken::consume`.
        let emac = unsafe { &mut *self.emac };
        let _ = emac.transmit(&buffer[..len]);
        result
    }
}

// =============================================================================
// Driver trait
// =============================================================================

impl<const RX: usize, const TX: usize, const BUF: usize> Driver for EmacDriver<'_, RX, TX, BUF> {
    type RxToken<'a>
        = EmacRxToken<'a, RX, TX, BUF>
    where
        Self: 'a;
    type TxToken<'a>
        = EmacTxToken<'a, RX, TX, BUF>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context<'_>) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        self.state.drv_rx_calls.fetch_add(1, Ordering::Relaxed);
        // SAFETY: see `EmacDriver` doc.
        let emac = unsafe { &mut *self.emac };

        if !emac.rx_available() {
            self.state.rx_waker.register(cx.waker());
            if !emac.rx_available() {
                return None;
            }
        }

        self.state.drv_rx_some.fetch_add(1, Ordering::Relaxed);
        Some((
            EmacRxToken {
                emac: self.emac,
                _marker: PhantomData,
            },
            EmacTxToken {
                emac: self.emac,
                _marker: PhantomData,
            },
        ))
    }

    fn transmit(&mut self, cx: &mut Context<'_>) -> Option<Self::TxToken<'_>> {
        self.state.drv_tx_calls.fetch_add(1, Ordering::Relaxed);
        // SAFETY: see `EmacDriver` doc.
        let emac = unsafe { &mut *self.emac };

        if !emac.tx_ready() {
            self.state.tx_waker.register(cx.waker());
            if !emac.tx_ready() {
                return None;
            }
        }

        self.state.drv_tx_some.fetch_add(1, Ordering::Relaxed);
        Some(EmacTxToken {
            emac: self.emac,
            _marker: PhantomData,
        })
    }

    fn link_state(&mut self, cx: &mut Context<'_>) -> LinkState {
        self.state.link_waker.register(cx.waker());
        self.state.link_state()
    }

    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::default();
        caps
    }

    fn hardware_address(&self) -> HardwareAddress {
        // SAFETY: see `EmacDriver` doc.
        let emac = unsafe { &*self.emac };
        HardwareAddress::Ethernet(emac.mac_address())
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_starts_link_down() {
        let s = EmacDriverState::new();
        assert!(matches!(s.link_state(), LinkState::Down));
    }

    #[test]
    fn state_link_set_up_then_down() {
        let s = EmacDriverState::new();
        s.set_link_up();
        assert!(matches!(s.link_state(), LinkState::Up));
        s.set_link_down();
        assert!(matches!(s.link_state(), LinkState::Down));
    }

    #[test]
    fn state_static_compatible() {
        static STATE: EmacDriverState = EmacDriverState::new();
        assert!(matches!(STATE.link_state(), LinkState::Down));
    }
}
