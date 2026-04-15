// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Embassy-net driver integration for ESP32 EMAC.
//!
//! This module provides an [`embassy_net_driver::Driver`] implementation
//! that wraps the EMAC, allowing it to be used with the `embassy-net`
//! TCP/IP stack.
//!
//! The entire module is gated behind the `embassy-net` Cargo feature.
//!
//! # Usage
//!
//! ```ignore
//! use embassy_net::{Config, Stack, StackResources};
//! use embassy_net_driver::LinkState;
//! use esp_emac::{Emac, EmacConfig};
//! use esp_emac::embassy::{EmacDriver, EmacDriverState};
//! use static_cell::StaticCell;
//!
//! static mut EMAC: Emac<10, 10, 1600> = Emac::new(EmacConfig::default());
//! static EMAC_STATE: EmacDriverState = EmacDriverState::new();
//! static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
//!
//! // Initialize EMAC first (GPIO/clock/PHY setup omitted).
//! let emac = unsafe { &mut EMAC };
//! emac.init().unwrap();
//! emac.enable();
//!
//! let driver = EmacDriver::new(emac, &EMAC_STATE);
//! let config = Config::dhcpv4(Default::default());
//! let seed = 0x1234_5678_9ABC_DEF0;
//! let (stack, runner) = embassy_net::new(
//!     driver,
//!     config,
//!     RESOURCES.init(StackResources::new()),
//!     seed,
//! );
//!
//! // In your EMAC interrupt handler:
//! // EMAC_STATE.on_interrupt();
//! ```
//!
//! # Interrupt Handling
//!
//! Call [`EmacDriverState::on_interrupt`] from the EMAC ISR to wake
//! async tasks waiting on RX/TX readiness.
//!
//! # Link State Updates
//!
//! Call [`EmacDriverState::set_link_up`] or [`EmacDriverState::set_link_down`]
//! from a periodic PHY polling task. The driver reports link state to
//! `embassy-net`, which controls DHCP and routing accordingly.

use core::cell::{Cell, RefCell};
use core::marker::PhantomData;
use core::task::Context;

use critical_section::Mutex;
use embassy_net_driver::{
    Capabilities, ChecksumCapabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken,
};

use crate::emac::Emac;

// =============================================================================
// Constants
// =============================================================================

/// Ethernet MTU (IP MTU + Ethernet header, excluding FCS).
///
/// Standard Ethernet frame: 1500 (IP) + 14 (header) = 1514 bytes.
/// embassy-net `max_transmission_unit` uses this value.
const ETHERNET_MTU: usize = 1514;

/// Maximum frame size for stack-allocated receive/transmit buffers.
///
/// Slightly larger than MTU to accommodate any padding.
const MAX_FRAME_SIZE: usize = 1600;

// =============================================================================
// Waker primitive (critical-section based)
// =============================================================================

/// Interrupt-safe waker storage.
///
/// Uses `critical_section::Mutex<RefCell<..>>` for ISR+task safety.
struct AtomicWaker {
    inner: Mutex<RefCell<Option<core::task::Waker>>>,
}

impl AtomicWaker {
    /// Create a new empty waker.
    const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(None)),
        }
    }

    /// Register a waker (called from async poll context).
    fn register(&self, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut slot = self.inner.borrow_ref_mut(cs);
            match &*slot {
                Some(existing) if existing.will_wake(waker) => {}
                _ => *slot = Some(waker.clone()),
            }
        });
    }

    /// Wake and clear the stored waker (safe to call from ISR).
    fn wake(&self) {
        let waker = critical_section::with(|cs| self.inner.borrow_ref_mut(cs).take());
        if let Some(w) = waker {
            w.wake();
        }
    }

    /// Check if a waker is currently registered (test helper).
    #[cfg(test)]
    fn is_registered(&self) -> bool {
        critical_section::with(|cs| self.inner.borrow_ref(cs).is_some())
    }
}

// SAFETY: All access goes through critical sections.
unsafe impl Send for AtomicWaker {}
// SAFETY: All access goes through critical sections.
unsafe impl Sync for AtomicWaker {}

// =============================================================================
// Embassy Driver State
// =============================================================================

/// Shared state for the embassy-net EMAC driver.
///
/// Stores wakers for RX, TX, and link state notifications.
/// Must be placed in a `static` so it is accessible from interrupt
/// handlers and async tasks.
pub struct EmacDriverState {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
    link_waker: AtomicWaker,
    link_up: Mutex<Cell<bool>>,
}

impl Default for EmacDriverState {
    fn default() -> Self {
        Self::new()
    }
}

impl EmacDriverState {
    /// Create a new driver state with link down.
    pub const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            link_waker: AtomicWaker::new(),
            link_up: Mutex::new(Cell::new(false)),
        }
    }

    /// Get the current link state.
    pub fn link_state(&self) -> LinkState {
        let up = critical_section::with(|cs| self.link_up.borrow(cs).get());
        if up {
            LinkState::Up
        } else {
            LinkState::Down
        }
    }

    /// Set the link state to up and wake any waiters.
    pub fn set_link_up(&self) {
        critical_section::with(|cs| self.link_up.borrow(cs).set(true));
        self.link_waker.wake();
    }

    /// Set the link state to down and wake any waiters.
    pub fn set_link_down(&self) {
        critical_section::with(|cs| self.link_up.borrow(cs).set(false));
        self.link_waker.wake();
    }

    /// Handle an EMAC interrupt.
    ///
    /// Reads the DMA status register, wakes the appropriate async tasks,
    /// and clears the handled interrupt flags.
    ///
    /// Call this from the EMAC ISR.
    pub fn on_interrupt(&self) {
        // SAFETY: This reads/writes MMIO registers. The caller is
        // responsible for ensuring the EMAC peripheral clock is enabled.
        let status = unsafe { crate::regs::dma::read(crate::regs::dma::DMASTATUS) };

        // Wake RX task on receive-complete or receive-buffer-unavailable.
        if status & (crate::regs::dma::status::RI | crate::regs::dma::status::RU) != 0 {
            self.rx_waker.wake();
        }

        // Wake TX task on transmit-complete or transmit-buffer-unavailable.
        if status & (crate::regs::dma::status::TI | crate::regs::dma::status::TU) != 0 {
            self.tx_waker.wake();
        }

        // On fatal bus error, wake both sides.
        if status & crate::regs::dma::status::FBI != 0 {
            self.rx_waker.wake();
            self.tx_waker.wake();
        }

        // Clear handled interrupt flags (write-1-to-clear).
        // SAFETY: Same MMIO safety as the read above.
        unsafe {
            crate::regs::dma::write(crate::regs::dma::DMASTATUS, status);
        }
    }
}

// SAFETY: All fields use critical-section-protected types.
unsafe impl Send for EmacDriverState {}
// SAFETY: All fields use critical-section-protected types.
unsafe impl Sync for EmacDriverState {}

// =============================================================================
// Embassy Driver Wrapper
// =============================================================================

/// Embassy-net driver for the ESP32 EMAC.
///
/// Wraps a mutable reference to [`Emac`] and shared [`EmacDriverState`].
/// Implements [`embassy_net_driver::Driver`] for use with `embassy-net`.
pub struct EmacDriver<'d, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    state: &'d EmacDriverState,
    _marker: PhantomData<&'d mut Emac<RX, TX, BUF>>,
}

impl<'d, const RX: usize, const TX: usize, const BUF: usize> EmacDriver<'d, RX, TX, BUF> {
    /// Create a new embassy-net driver.
    ///
    /// # Arguments
    ///
    /// * `emac` - Initialized EMAC (must be in its final memory location)
    /// * `state` - Shared driver state (typically a `static`)
    pub fn new(emac: &'d mut Emac<RX, TX, BUF>, state: &'d EmacDriverState) -> Self {
        Self {
            emac: emac as *mut Emac<RX, TX, BUF>,
            state,
            _marker: PhantomData,
        }
    }

    /// Get a reference to the shared driver state.
    pub fn state(&self) -> &EmacDriverState {
        self.state
    }
}

// =============================================================================
// RX / TX Tokens
// =============================================================================

/// Receive token for a single packet.
///
/// Created by [`EmacDriver::receive`] and consumed by the embassy-net
/// stack to read one frame.
pub struct EmacRxToken<'d, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    _marker: PhantomData<&'d mut Emac<RX, TX, BUF>>,
}

impl<const RX: usize, const TX: usize, const BUF: usize> RxToken for EmacRxToken<'_, RX, TX, BUF> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buffer = [0u8; MAX_FRAME_SIZE];

        // SAFETY: The raw pointer is valid for the driver lifetime.
        // Tokens are created and consumed within a single poll cycle
        // by the embassy-net stack.
        let emac = unsafe { &mut *self.emac };

        let len = match emac.receive(&mut buffer) {
            Ok(Some(n)) => n,
            _ => 0,
        };
        f(&mut buffer[..len])
    }
}

/// Transmit token for a single packet.
///
/// Created by [`EmacDriver::transmit`] or [`EmacDriver::receive`]
/// and consumed by the embassy-net stack to send one frame.
pub struct EmacTxToken<'d, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    _marker: PhantomData<&'d mut Emac<RX, TX, BUF>>,
}

impl<const RX: usize, const TX: usize, const BUF: usize> TxToken for EmacTxToken<'_, RX, TX, BUF> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let len = len.min(MAX_FRAME_SIZE);
        let mut buffer = [0u8; MAX_FRAME_SIZE];
        let result = f(&mut buffer[..len]);

        // SAFETY: The raw pointer is valid for the driver lifetime.
        // Tokens are created and consumed within a single poll cycle.
        let emac = unsafe { &mut *self.emac };

        let _ = emac.transmit(&buffer[..len]);
        result
    }
}

// =============================================================================
// Driver Implementation
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
        // SAFETY: The raw pointer is valid for the driver lifetime.
        let emac = unsafe { &mut *self.emac };

        if !emac.rx_available() {
            self.state.rx_waker.register(cx.waker());
            // Double-check after registering waker to avoid missed notifications.
            if !emac.rx_available() {
                return None;
            }
        }

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
        // SAFETY: The raw pointer is valid for the driver lifetime.
        let emac = unsafe { &mut *self.emac };

        // Check if at least one single-buffer frame can be sent.
        if !emac.can_transmit(1) {
            self.state.tx_waker.register(cx.waker());
            // Double-check after registering waker.
            if !emac.can_transmit(1) {
                return None;
            }
        }

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
        caps.max_transmission_unit = ETHERNET_MTU;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::default();
        caps
    }

    fn hardware_address(&self) -> HardwareAddress {
        // SAFETY: The raw pointer is valid for the driver lifetime.
        let emac = unsafe { &*self.emac };
        HardwareAddress::Ethernet(emac.mac_address())
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins};

    /// Helper: create a test-friendly EmacConfig.
    fn test_config() -> EmacConfig {
        EmacConfig {
            clock: RmiiClockConfig::InternalApll {
                gpio: ClkGpio::Gpio17,
            },
            pins: RmiiPins::default(),
        }
    }

    // ── EmacDriverState ────────────────────────────────────────────────

    #[test]
    fn state_new_link_down() {
        let state = EmacDriverState::new();
        assert!(state.link_state() == LinkState::Down);
    }

    #[test]
    fn state_set_link_up() {
        let state = EmacDriverState::new();
        state.set_link_up();
        assert!(state.link_state() == LinkState::Up);
    }

    #[test]
    fn state_set_link_down() {
        let state = EmacDriverState::new();
        state.set_link_up();
        assert!(state.link_state() == LinkState::Up);
        state.set_link_down();
        assert!(state.link_state() == LinkState::Down);
    }

    #[test]
    fn state_link_toggle() {
        let state = EmacDriverState::new();
        for _ in 0..5 {
            state.set_link_up();
            assert!(state.link_state() == LinkState::Up);
            state.set_link_down();
            assert!(state.link_state() == LinkState::Down);
        }
    }

    #[test]
    fn state_static_compatible() {
        // Verify EmacDriverState can live in a static.
        static STATE: EmacDriverState = EmacDriverState::new();
        assert!(STATE.link_state() == LinkState::Down);
    }

    // ── AtomicWaker ────────────────────────────────────────────────────

    #[test]
    fn waker_initially_empty() {
        let waker = AtomicWaker::new();
        assert!(!waker.is_registered());
    }

    #[test]
    fn waker_wake_without_register_is_noop() {
        let waker = AtomicWaker::new();
        waker.wake(); // should not panic
        assert!(!waker.is_registered());
    }

    #[test]
    fn waker_register_and_wake() {
        use core::task::{RawWaker, RawWakerVTable, Waker};
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;

        struct Counter(AtomicUsize);

        fn make_waker(counter: Arc<Counter>) -> Waker {
            fn clone_fn(ptr: *const ()) -> RawWaker {
                // SAFETY: ptr is from Arc::into_raw in this test.
                let arc = unsafe { Arc::from_raw(ptr as *const Counter) };
                let cloned = arc.clone();
                core::mem::forget(arc);
                RawWaker::new(Arc::into_raw(cloned) as *const (), &VTABLE)
            }
            fn wake_fn(ptr: *const ()) {
                // SAFETY: ptr is from Arc::into_raw in this test.
                let arc = unsafe { Arc::from_raw(ptr as *const Counter) };
                arc.0.fetch_add(1, Ordering::SeqCst);
            }
            fn wake_by_ref_fn(ptr: *const ()) {
                // SAFETY: ptr is from Arc::into_raw in this test.
                let arc = unsafe { Arc::from_raw(ptr as *const Counter) };
                arc.0.fetch_add(1, Ordering::SeqCst);
                core::mem::forget(arc);
            }
            fn drop_fn(ptr: *const ()) {
                // SAFETY: ptr is from Arc::into_raw in this test.
                unsafe {
                    Arc::from_raw(ptr as *const Counter);
                }
            }
            static VTABLE: RawWakerVTable =
                RawWakerVTable::new(clone_fn, wake_fn, wake_by_ref_fn, drop_fn);
            let raw = RawWaker::new(Arc::into_raw(counter) as *const (), &VTABLE);
            // SAFETY: raw is built from a valid vtable and pointer.
            unsafe { Waker::from_raw(raw) }
        }

        let counter = Arc::new(Counter(AtomicUsize::new(0)));
        let waker = make_waker(counter.clone());

        let aw = AtomicWaker::new();
        aw.register(&waker);
        assert!(aw.is_registered());

        aw.wake();
        assert_eq!(counter.0.load(Ordering::SeqCst), 1);
        assert!(!aw.is_registered());

        // Double wake is a no-op (waker already consumed).
        aw.wake();
        assert_eq!(counter.0.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn waker_register_overwrites_previous() {
        use core::task::{RawWaker, RawWakerVTable, Waker};
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;

        struct Counter(AtomicUsize);

        fn make_waker(counter: Arc<Counter>) -> Waker {
            fn clone_fn(ptr: *const ()) -> RawWaker {
                // SAFETY: ptr is from Arc::into_raw in this test.
                let arc = unsafe { Arc::from_raw(ptr as *const Counter) };
                let cloned = arc.clone();
                core::mem::forget(arc);
                RawWaker::new(Arc::into_raw(cloned) as *const (), &VTABLE)
            }
            fn wake_fn(ptr: *const ()) {
                // SAFETY: ptr is from Arc::into_raw in this test.
                let arc = unsafe { Arc::from_raw(ptr as *const Counter) };
                arc.0.fetch_add(1, Ordering::SeqCst);
            }
            fn wake_by_ref_fn(ptr: *const ()) {
                // SAFETY: ptr is from Arc::into_raw in this test.
                let arc = unsafe { Arc::from_raw(ptr as *const Counter) };
                arc.0.fetch_add(1, Ordering::SeqCst);
                core::mem::forget(arc);
            }
            fn drop_fn(ptr: *const ()) {
                // SAFETY: ptr is from Arc::into_raw in this test.
                unsafe {
                    Arc::from_raw(ptr as *const Counter);
                }
            }
            static VTABLE: RawWakerVTable =
                RawWakerVTable::new(clone_fn, wake_fn, wake_by_ref_fn, drop_fn);
            let raw = RawWaker::new(Arc::into_raw(counter) as *const (), &VTABLE);
            // SAFETY: raw is built from a valid vtable and pointer.
            unsafe { Waker::from_raw(raw) }
        }

        let counter1 = Arc::new(Counter(AtomicUsize::new(0)));
        let counter2 = Arc::new(Counter(AtomicUsize::new(0)));
        let waker1 = make_waker(counter1.clone());
        let waker2 = make_waker(counter2.clone());

        let aw = AtomicWaker::new();
        aw.register(&waker1);
        aw.register(&waker2);
        aw.wake();

        // Only the second waker should have been woken.
        assert_eq!(counter1.0.load(Ordering::SeqCst), 0);
        assert_eq!(counter2.0.load(Ordering::SeqCst), 1);
    }

    // ── Capabilities ───────────────────────────────────────────────────

    #[test]
    fn capabilities_mtu() {
        let mut emac: Emac<4, 4, 1600> = Emac::new(test_config());
        let state = EmacDriverState::new();
        let driver = EmacDriver::new(&mut emac, &state);

        let caps = driver.capabilities();
        assert_eq!(caps.max_transmission_unit, ETHERNET_MTU);
        assert_eq!(caps.max_burst_size, Some(1));
    }

    #[test]
    fn hardware_address_returns_mac() {
        let mut emac: Emac<4, 4, 1600> = Emac::new(test_config());
        let mac = [0x02, 0x00, 0x00, 0x12, 0x34, 0x56];
        emac.set_mac_address(mac);

        let state = EmacDriverState::new();
        let driver = EmacDriver::new(&mut emac, &state);

        assert!(driver.hardware_address() == HardwareAddress::Ethernet(mac));
    }

    #[test]
    fn hardware_address_zero_default() {
        let mut emac: Emac<4, 4, 1600> = Emac::new(test_config());
        let state = EmacDriverState::new();
        let driver = EmacDriver::new(&mut emac, &state);

        assert!(driver.hardware_address() == HardwareAddress::Ethernet([0; 6]));
    }

    #[test]
    fn state_accessor() {
        let mut emac: Emac<4, 4, 1600> = Emac::new(test_config());
        let state = EmacDriverState::new();
        let driver = EmacDriver::new(&mut emac, &state);

        assert!(driver.state().link_state() == LinkState::Down);
    }

    // ── Constants ──────────────────────────────────────────────────────

    #[test]
    fn ethernet_mtu_is_standard() {
        // Standard Ethernet: 1500 IP + 14 header = 1514.
        assert_eq!(ETHERNET_MTU, 1514);
    }

    #[test]
    fn max_frame_size_ge_mtu() {
        assert!(MAX_FRAME_SIZE >= ETHERNET_MTU);
    }

    // ── LinkState ──────────────────────────────────────────────────────

    #[test]
    fn link_state_equality() {
        assert!(LinkState::Up == LinkState::Up);
        assert!(LinkState::Down == LinkState::Down);
        assert!(LinkState::Up != LinkState::Down);
    }
}
