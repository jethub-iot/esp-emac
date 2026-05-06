// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Native embassy-net driver for the ESP32 EMAC.
//!
//! Wraps [`crate::Emac`] directly (no `ph_esp32_mac::EmbassyEmac` proxy).
//! [`EmacDriverState`] holds the wakers, link cache, and ISR counters
//! and is intended to live in `static` storage so the EMAC ISR can
//! reach it.
//!
//! # Lifetime alignment
//!
//! [`Emac`] and [`EmacDriverState`] play different roles, with
//! different uniqueness requirements:
//!
//! - [`Emac`] drives a single hardware peripheral. The ESP32 has
//!   exactly one built-in EMAC and `Emac::init` touches global MMIO,
//!   so at most one initialized `Emac` instance can be active on a
//!   running device. Place it in `static mut` storage and take the
//!   `&'static mut` once at bring-up via
//!   `unsafe { &mut *core::ptr::addr_of_mut!(EMAC) }`. `Emac::new`
//!   is a `const fn`, so the value lives in BSS — no runtime stack
//!   temporary, deterministic on cold boot. (See the *Usage* section
//!   below for why a `StaticCell::init(EmacDefault::new(..))`
//!   wrapper is *not* recommended at the default ring sizing.)
//! - [`EmacDriverState`] is **not** a strict singleton. Multiple
//!   instances are fine — host-side tests construct one per test, and
//!   sequential re-initialization with a fresh state on the same
//!   peripheral is allowed. The constraint is alignment: the
//!   `EmacDriverState` whose [`handle_emac_interrupt`] runs from the
//!   ISR must be the same instance you pass to [`EmacDriver::new`]
//!   alongside the `Emac`. If those two references disagree, RX/TX
//!   wakers fire against the wrong state and the stack stalls.
//!
//! [`EmacDriver`] then ties the pair together. The borrow checker
//! enforces that **at most one** driver holds the `&'d mut Emac<...>`
//! at any given moment. Sequential reuse is allowed — once the borrow
//! ends (driver dropped, scope exited) the same `Emac` can be paired
//! with a fresh driver again, which is what the unit tests in this
//! module already exercise.
//!
//! # Recovery from task respawn
//!
//! `Emac::init` is a one-shot: if the task that owns the
//! `&'static mut EmacDefault` panics and is respawned by the executor,
//! the static EMAC retains state from the previous run. Calling
//! `init` a second time returns [`EmacError::AlreadyInitialized`] and
//! does nothing. The reborrowed `&'static mut` is still valid — the
//! peripheral is still configured — but the DMA engine may have
//! stopped mid-operation (descriptors marked owned by the engine,
//! TX FIFO partially drained), and the driver state in
//! [`EmacDriverState`] no longer matches the in-flight wakers from
//! the previous run.
//!
//! Recovery sequence in the respawned task:
//!
//! ```ignore
//! use esp_hal::delay::Delay;
//!
//! // Reborrow — same EMAC, still post-init from prior run.
//! let emac = unsafe { &mut *core::ptr::addr_of_mut!(EMAC) };
//! // Tear down the running engine and clear DMA state. `stop()`
//! // takes a `&mut impl DelayNs` for the TX-FIFO flush poll.
//! // It is idempotent on `Initialized` (returns Ok(())) and rejects
//! // an `Uninitialized` driver with `EmacError::NotInitialized` —
//! // neither matters here because the prior task left the EMAC in
//! // `Running`. `Err(EmacError::TxFlushTimeout)` means teardown
//! // still completed (state is back at `Initialized`); the warning
//! // is recoverable, so swallow it.
//! let mut delay = Delay::new();
//! let _ = emac.stop(&mut delay);
//! // Restart fresh. The peripheral keeps its already-programmed
//! // pins, clocks, and MAC address — only the DMA rings need to
//! // come back up.
//! emac.start()?;
//! ```
//!
//! Do **not** call `init()` a second time hoping it will reset the
//! peripheral — it won't, and the error swallows silently in code
//! that ignores the `Result`. Use the explicit `stop()` + `start()`
//! cycle above.
//!
//! [`handle_emac_interrupt`]: EmacDriverState::handle_emac_interrupt
//! [`EmacError::AlreadyInitialized`]: crate::EmacError::AlreadyInitialized
//!
//! # Usage
//!
//! The driver is non-functional until the EMAC ISR services
//! `DMASTATUS` and wakes the RX/TX tasks. The ISR body must call
//! [`EmacDriverState::handle_emac_interrupt`] (or the lower-level
//! pair [`crate::Emac::handle_interrupt`] +
//! [`EmacDriverState::on_interrupt_status`]) — without that, RX and
//! TX block forever in `Driver::receive` / `Driver::transmit` waiting
//! on wakers that nothing pokes.
//!
//! ```ignore
//! use esp_emac::{
//!     EmacConfig, RmiiClockConfig, RmiiPins, ClkGpio, XtalFreq,
//!     EmacDefault,
//!     embassy::{EmacDefaultDriver, EmacDriverState},
//! };
//! use esp_hal::interrupt::{InterruptHandler, Priority};
//!
//! // `EmacDefault::new` is a `const fn`, so the EMAC value is built at
//! // compile time and lives in BSS — zero runtime stack involvement on
//! // boot. The default ring sizing is currently 10 RX / 10 TX /
//! // 1600-byte buffers (~32 KiB), sourced from `DEFAULT_RX` /
//! // `DEFAULT_TX` / `DEFAULT_BUF`. A `StaticCell::init(EmacDefault::new(..))`
//! // pattern would risk landing that 32 KiB on the caller's stack
//! // before the move into static storage; the `static mut` form is
//! // smaller, deterministic and avoids that hazard.
//! static mut EMAC: EmacDefault = EmacDefault::new(EmacConfig {
//!     clock: RmiiClockConfig::InternalApll {
//!         gpio: ClkGpio::Gpio17,
//!         xtal: XtalFreq::Mhz40,
//!     },
//!     pins: RmiiPins { mdc: 23, mdio: 18 },
//! });
//! static EMAC_STATE: EmacDriverState = EmacDriverState::new();
//!
//! // 1. ISR — must service DMASTATUS and wake stack tasks. The
//! //    `EMAC_STATE` it touches has to be the same instance the
//! //    driver is paired with below.
//! #[esp_hal::handler(priority = Priority::Priority1)]
//! fn emac_isr() {
//!     EMAC_STATE.handle_emac_interrupt();
//! }
//!
//! // 2. Bring-up + driver wiring.
//! # fn example() -> Result<(), esp_emac::EmacError> {
//! # let mut delay = esp_hal::delay::Delay::new();
//! // SAFETY: `EMAC` is touched only here — single owner — so no aliasing.
//! let emac = unsafe { &mut *core::ptr::addr_of_mut!(EMAC) };
//! emac.set_mac_address([0x00, 0x70, 0x07, 0x24, 0x3B, 0x87]);
//! emac.init(&mut delay)?;
//! // ... PHY init + link wait + set_speed/set_duplex omitted ...
//! emac.bind_interrupt(InterruptHandler::new(emac_isr, Priority::Priority1));
//! emac.start()?;
//! // `EmacDefaultDriver` is a type alias whose inherent `new` is the
//! // same `EmacDriver::new` constructor — keeps the call site free of
//! // the const-generic ceremony (currently `<10, 10, 1600>`, sourced
//! // from `DEFAULT_RX` / `DEFAULT_TX` / `DEFAULT_BUF`).
//! let driver = EmacDefaultDriver::new(emac, &EMAC_STATE);
//! // Hand `driver` to embassy_net::new() / Stack.
//! # Ok(()) }
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

use crate::emac::{Emac, DEFAULT_BUF, DEFAULT_RX, DEFAULT_TX, SMALL_RX, SMALL_TX};
use crate::interrupt::InterruptStatus;

/// Diagnostic snapshot of the `Driver::receive` / `Driver::transmit`
/// path: how many times embassy-net asked for a token, how many of
/// those calls actually had a frame, and how many frames the tokens
/// failed to push to / pull from the EMAC.
#[derive(Debug, Clone, Copy, Default)]
pub struct DriverCounters {
    /// Calls to `Driver::receive`.
    pub rx_calls: u32,
    /// Calls that returned a non-empty token pair (frame available).
    pub rx_some: u32,
    /// Frames silently dropped in `EmacRxToken::consume` because the
    /// underlying `Emac::receive` returned `Err(_)` or `Ok(None)` after
    /// the driver had already handed out a token. Indicates either an
    /// errored frame (CRC, oversize) or a race where another path
    /// consumed the descriptor first.
    pub rx_dropped: u32,
    /// Calls to `Driver::transmit`.
    pub tx_calls: u32,
    /// Calls that returned a token (TX path was ready).
    pub tx_some: u32,
    /// Frames silently dropped in `EmacTxToken::consume` because
    /// `Emac::transmit` returned `Err(_)` after the driver had already
    /// handed out a token. Typical cause: descriptor ring exhausted
    /// between the readiness check and the actual push.
    pub tx_dropped: u32,
}

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

/// Standard Ethernet MTU (IP MTU + L2 header). Upper bound on the value
/// the driver advertises to embassy-net — see
/// `EmacDriver::effective_mtu` for the per-instance value, which caps
/// this against the physical TX ring capacity.
const ETH_MTU: usize = 1514;

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
    drv_rx_dropped: AtomicU32,
    drv_tx_calls: AtomicU32,
    drv_tx_some: AtomicU32,
    drv_tx_dropped: AtomicU32,
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
            drv_rx_dropped: AtomicU32::new(0),
            drv_tx_calls: AtomicU32::new(0),
            drv_tx_some: AtomicU32::new(0),
            drv_tx_dropped: AtomicU32::new(0),
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

    /// Diagnostic counters from `Driver::receive` / `Driver::transmit`
    /// and the matching tokens.
    pub fn driver_counters(&self) -> DriverCounters {
        DriverCounters {
            rx_calls: self.drv_rx_calls.load(Ordering::Relaxed),
            rx_some: self.drv_rx_some.load(Ordering::Relaxed),
            rx_dropped: self.drv_rx_dropped.load(Ordering::Relaxed),
            tx_calls: self.drv_tx_calls.load(Ordering::Relaxed),
            tx_some: self.drv_tx_some.load(Ordering::Relaxed),
            tx_dropped: self.drv_tx_dropped.load(Ordering::Relaxed),
        }
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

    /// Read the DMA status register, clear the interrupts, and wake
    /// any tasks waiting on RX or TX.
    ///
    /// Does **not** wake `link_waker` — link state isn't reflected in
    /// `DMASTATUS` and is updated separately by whatever PHY-polling
    /// task calls [`set_link_up`](Self::set_link_up) /
    /// [`set_link_down`](Self::set_link_down). That path takes care
    /// of waking link-state observers itself.
    ///
    /// Intended to be called from the EMAC ISR. Touches only memory-
    /// mapped EMAC registers and the embedded wakers, so there is no
    /// aliasing concern with the [`EmacDriver`] holding a raw pointer
    /// to the [`Emac`] state.
    pub fn handle_emac_interrupt(&self) {
        let dmastat = crate::regs::dma::BASE + crate::regs::dma::DMASTATUS;
        // SAFETY: DMASTATUS is a known-valid 32-bit memory-mapped register.
        let raw = unsafe { core::ptr::read_volatile(dmastat as *const u32) };
        let status = InterruptStatus::from_raw(raw);
        // Write-1-to-clear using the raw snapshot, masked against
        // `ALL_INTERRUPTS` so only the W1C interrupt bits are written
        // back. This still catches every asserted W1C bit — including
        // ones outside `InterruptStatus` such as `ERI` (bit 14), `ETI`
        // (bit 10), `RWT` (bit 9), `TJT` (bit 3) — but excludes the
        // read-only fields (`RS`/`TS`/`EB`/`MMC`/`PMT`/`TTI`) so we
        // never write garbage at addresses the hardware doesn't expect.
        // Round-tripping through `to_raw()` would silently drop those
        // bits and risk an interrupt storm.
        // SAFETY: same address; masked write hits only W1C bits.
        unsafe {
            core::ptr::write_volatile(
                dmastat as *mut u32,
                raw & crate::regs::dma::status::ALL_INTERRUPTS,
            )
        };

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
        // Early Receive Interrupt (ERI, bit 14 of DMASTATUS — distinct
        // from ETI, the Early Transmit Interrupt at bit 10) isn't
        // surfaced through `InterruptStatus`, so check the raw flag
        // against the canonical `regs::dma::status::ERI` constant
        // rather than a magic shift.
        if (raw & crate::regs::dma::status::ERI) != 0 {
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
/// # Concurrent ownership
///
/// At most **one** `EmacDriver` can hold the `&'d mut Emac<...>` at a
/// time. The borrow checker enforces that through the `&'d mut`
/// argument to [`Self::new`] — concurrent aliasing is impossible.
/// Sequential reuse is fine: once a driver is dropped, the same
/// `Emac` can be paired with a fresh driver again. The unit tests in
/// this module exercise that pattern.
///
/// The companion [`EmacDriverState`] is **not** a strict singleton —
/// see the module-level *Lifetime alignment* section. The constraint
/// is that whichever instance the EMAC ISR's
/// [`EmacDriverState::handle_emac_interrupt`] runs against must be
/// the same one passed here as `state`.
///
/// For the default ring sizing the
/// [`EmacDefaultDriver<'d>`](EmacDefaultDriver) alias removes the need
/// to repeat the const generics in `embassy_executor::task` signatures.
///
/// # Safety
///
/// The pointer is dereferenced in `Driver` impl methods. The lifetime
/// `'d` ensures the underlying `Emac` outlives the driver, but the raw
/// pointer means **mutable aliasing** would be unsound. The
/// at-most-one-concurrent-driver invariant above is what keeps that
/// aliasing impossible in practice.
pub struct EmacDriver<'d, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    state: &'d EmacDriverState,
    _marker: PhantomData<&'d mut Emac<RX, TX, BUF>>,
}

// SAFETY contract for `unsafe impl Send`:
//
// `EmacDriver` carries a `*mut Emac<...>` (raw pointers are not auto-Send),
// but the manual impl is sound under the following invariants — break any
// one of them and the impl becomes unsound, so revisit it together with
// any change that touches `Emac`'s field layout or the ISR data path:
//
// 1. Single ownership. Exactly one `EmacDriver` exists per `Emac`
//    instance for the lifetime of `'d`. `EmacDriver::new` consumes a
//    `&'d mut Emac<...>`, which the borrow checker enforces as long
//    as the pointer isn't laundered through other unsafe code.
// 2. ISR-side access through `EmacDriverState` only touches MMIO
//    (`DMASTATUS`) and `AtomicU32` counters — *not* the `Emac` struct
//    behind the raw pointer. So the ISR is not a concurrent reader
//    of the data the `Driver` impl mutates.
// 3. The pointee `Emac<RX, TX, BUF>` is itself `Send`. The raw pointer
//    hides auto-trait inference, so without an explicit bound a
//    future `Cell<X>` / `Rc<X>` / `MutexGuard<'_, X>` inside `Emac`
//    would silently leave this impl claiming `Send`. The
//    `where Emac<RX, TX, BUF>: Send` clause below promotes that
//    invariant from documentation to a compile-time check: such a
//    refactor will fail to compile here instead of producing
//    unsound `EmacDriver: Send`.
unsafe impl<const RX: usize, const TX: usize, const BUF: usize> Send for EmacDriver<'_, RX, TX, BUF> where
    Emac<RX, TX, BUF>: Send
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

    /// Effective MTU advertised to embassy-net and used as the
    /// readiness threshold in `Driver::transmit`.
    ///
    /// Capped by the physical TX ring capacity (`TX * BUF`) so the
    /// driver never advertises — and never gates on — a frame size
    /// the engine couldn't actually push. On normal rings (e.g.
    /// `TX=10, BUF=1600`) this returns the standard Ethernet MTU
    /// of `1514`. On undersized rings (`TX * BUF < 1514`) it shrinks
    /// to `TX * BUF`, so small frames still flow even though full-MTU
    /// frames are physically impossible.
    pub const fn effective_mtu() -> usize {
        let ring_capacity = TX * BUF;
        if ring_capacity < ETH_MTU {
            ring_capacity
        } else {
            ETH_MTU
        }
    }
}

// =============================================================================
// Convenience type aliases
// =============================================================================

/// Driver for the [`crate::EmacDefault`] ring sizing.
///
/// Sourced from the same [`DEFAULT_RX`] / [`DEFAULT_TX`] /
/// [`DEFAULT_BUF`] constants as `EmacDefault`, so the two aliases
/// stay paired even if the canonical sizing is retuned. The
/// `embassy_executor::task` signature for the `net_task` runner can
/// then read `Runner<'static, EmacDefaultDriver<'static>>` instead
/// of repeating the const generics at every call site.
pub type EmacDefaultDriver<'d> = EmacDriver<'d, DEFAULT_RX, DEFAULT_TX, DEFAULT_BUF>;

/// Driver for the [`crate::EmacSmall`] ring sizing.
///
/// See [`EmacDefaultDriver`] for the rationale.
pub type EmacSmallDriver<'d> = EmacDriver<'d, SMALL_RX, SMALL_TX, DEFAULT_BUF>;

// =============================================================================
// RX / TX tokens
// =============================================================================

/// embassy-net RX token — copies one received frame on `consume`.
pub struct EmacRxToken<'a, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    state: &'a EmacDriverState,
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
        match emac.receive(&mut buffer) {
            Ok(Some(n)) => f(&mut buffer[..n]),
            // No frame after we already handed out a token — either an
            // error path (FrameError, BufferTooSmall: descriptor was
            // recycled by the engine) or a race where another caller
            // consumed it. Bump `rx_dropped` so the drop is observable
            // and pass an empty slice to satisfy the `RxToken` contract.
            Ok(None) | Err(_) => {
                self.state.drv_rx_dropped.fetch_add(1, Ordering::Relaxed);
                f(&mut [])
            }
        }
    }
}

/// embassy-net TX token — submits one frame on `consume`.
pub struct EmacTxToken<'a, const RX: usize, const TX: usize, const BUF: usize> {
    emac: *mut Emac<RX, TX, BUF>,
    state: &'a EmacDriverState,
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
        if emac.transmit(&buffer[..len]).is_err() {
            // `embassy-net-driver`'s `TxToken::consume` has no fallible
            // return, so a failed push silently drops the frame. Bump
            // `tx_dropped` for diagnostics.
            self.state.drv_tx_dropped.fetch_add(1, Ordering::Relaxed);
        }
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
                state: self.state,
                _marker: PhantomData,
            },
            EmacTxToken {
                emac: self.emac,
                state: self.state,
                _marker: PhantomData,
            },
        ))
    }

    fn transmit(&mut self, cx: &mut Context<'_>) -> Option<Self::TxToken<'_>> {
        self.state.drv_tx_calls.fetch_add(1, Ordering::Relaxed);
        // SAFETY: see `EmacDriver` doc.
        let emac = unsafe { &mut *self.emac };

        // Gate on capacity for a worst-case MTU-sized frame, not just
        // "≥ 1 free descriptor". A frame larger than `BUF` consumes
        // `len.div_ceil(BUF)` descriptors, so on rings where `BUF < MTU`
        // a single-descriptor readiness check would let the driver hand
        // out a token that `EmacTxToken::consume` then can't actually
        // push, silently dropping the frame.
        //
        // `effective_mtu()` is capped by `TX * BUF`, so on undersized
        // rings we still gate on something the engine can transmit
        // (smaller frames will fit) instead of permanently returning
        // `None` for a 1514-byte target the ring can never hold.
        let mtu = Self::effective_mtu();
        if !emac.can_transmit(mtu) {
            self.state.tx_waker.register(cx.waker());
            if !emac.can_transmit(mtu) {
                return None;
            }
        }

        self.state.drv_tx_some.fetch_add(1, Ordering::Relaxed);
        Some(EmacTxToken {
            emac: self.emac,
            state: self.state,
            _marker: PhantomData,
        })
    }

    fn link_state(&mut self, cx: &mut Context<'_>) -> LinkState {
        self.state.link_waker.register(cx.waker());
        self.state.link_state()
    }

    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        // Advertise the value the driver can actually deliver (capped
        // by ring capacity), not a fixed Ethernet MTU.
        caps.max_transmission_unit = Self::effective_mtu();
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

    // ── Driver wrapper (host-side static behaviour) ──────────────

    fn test_emac() -> Emac<10, 10, 1600> {
        use crate::config::{ClkGpio, EmacConfig, RmiiClockConfig, RmiiPins, XtalFreq};

        Emac::new(EmacConfig {
            clock: RmiiClockConfig::InternalApll {
                gpio: ClkGpio::Gpio17,
                xtal: XtalFreq::Mhz40,
            },
            pins: RmiiPins { mdc: 23, mdio: 18 },
        })
    }

    #[test]
    fn driver_capabilities_advertise_mtu_and_burst() {
        let mut emac = test_emac();
        let state = EmacDriverState::new();
        let driver = EmacDriver::new(&mut emac, &state);

        let caps = driver.capabilities();
        // 10 × 1600 ring fits a full Ethernet frame, so `effective_mtu`
        // collapses to the standard ETH_MTU.
        assert_eq!(caps.max_transmission_unit, ETH_MTU);
        assert_eq!(caps.max_transmission_unit, 1514);
        // Single-frame burst — the driver hands out one TX token at a
        // time, so the stack should not pipeline more than one frame.
        assert_eq!(caps.max_burst_size, Some(1));
    }

    #[test]
    fn effective_mtu_caps_to_ring_capacity() {
        // Standard configuration: ring is plenty large, full ETH MTU.
        assert_eq!(EmacDriver::<10, 10, 1600>::effective_mtu(), ETH_MTU);
        assert_eq!(EmacDriver::<4, 4, 1600>::effective_mtu(), ETH_MTU);
        // Undersized ring: `TX * BUF = 1024` < 1514. We must NOT advertise
        // 1514 — the engine can't transmit that. Capped to ring capacity.
        assert_eq!(EmacDriver::<2, 2, 512>::effective_mtu(), 1024);
        // Edge: exactly equal to ETH_MTU.
        assert_eq!(EmacDriver::<1, 1, 1514>::effective_mtu(), ETH_MTU);
        // One byte short.
        assert_eq!(EmacDriver::<1, 1, 1513>::effective_mtu(), 1513);
    }

    #[test]
    fn driver_hardware_address_reflects_cached_mac() {
        let mut emac = test_emac();
        // Before any `set_mac_address`, the cached value is the zero
        // address — the bring-up code is expected to programme one
        // before `init` reaches the address-filter step.
        {
            let state = EmacDriverState::new();
            let driver = EmacDriver::new(&mut emac, &state);
            let HardwareAddress::Ethernet(mac) = driver.hardware_address() else {
                panic!("expected Ethernet hardware address");
            };
            assert_eq!(mac, [0u8; 6]);
        }

        // Cache a MAC; the driver should reflect it on the next read.
        let custom = [0xF0, 0x57, 0x8D, 0x01, 0x04, 0xE3];
        emac.set_mac_address(custom);

        let state = EmacDriverState::new();
        let driver = EmacDriver::new(&mut emac, &state);
        let HardwareAddress::Ethernet(mac) = driver.hardware_address() else {
            panic!("expected Ethernet hardware address");
        };
        assert_eq!(mac, custom);
    }
}
