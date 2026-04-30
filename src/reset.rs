// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! DMA software reset state machine.
//!
//! Wraps the `DMABUSMODE.SWR` bit-poll loop in a small struct that takes
//! a [`DelayNs`] implementation. Note that
//! [`embedded_hal::delay::DelayNs`] is a **blocking** delay trait — the
//! poll loop will busy-wait the calling task until the controller
//! self-clears or the timeout expires. Callers from async contexts
//! should either accept that block (the reset finishes in a few
//! microseconds on real hardware) or pass a `DelayNs` implementation
//! whose `delay_us` yields to the executor.
//!
//! For a true async variant — using `embedded_hal_async::delay::DelayNs`
//! so each poll-step `.await`s and yields control back to the executor —
//! enable the `async` cargo feature and use
//! `crate::reset::async_impl::AsyncResetController`, defined in the
//! cfg-gated `async_impl` submodule.

use embedded_hal::delay::DelayNs;

use crate::regs::dma::{self, bus_mode, DMABUSMODE};

/// Default soft-reset timeout (matches ESP-IDF / ph-esp32-mac).
pub const SOFT_RESET_TIMEOUT_MS: u32 = 100;
/// Polling interval while waiting for `DMABUSMODE.SWR` to self-clear.
pub const RESET_POLL_INTERVAL_US: u32 = 100;

/// Reset failure cause.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetError {
    /// `DMABUSMODE.SWR` did not self-clear within the configured timeout.
    Timeout,
}

/// Owns a [`DelayNs`] implementation and exposes the soft-reset routine.
pub struct ResetController<D: DelayNs> {
    delay: D,
    timeout_ms: u32,
}

impl<D: DelayNs> ResetController<D> {
    /// Build a controller with the [`SOFT_RESET_TIMEOUT_MS`] default.
    pub fn new(delay: D) -> Self {
        Self {
            delay,
            timeout_ms: SOFT_RESET_TIMEOUT_MS,
        }
    }

    /// Build a controller with a caller-chosen timeout.
    pub fn with_timeout(delay: D, timeout_ms: u32) -> Self {
        Self { delay, timeout_ms }
    }

    /// Issue the DMA software reset and wait for `DMABUSMODE.SWR` to
    /// self-clear. Returns [`ResetError::Timeout`] if it does not happen
    /// within `timeout_ms`. The reset clears the entire DMA + MAC core to
    /// its hardware-default state.
    pub fn soft_reset(&mut self) -> Result<(), ResetError> {
        // Setting the SWR bit triggers the reset; the bit auto-clears when
        // the controller is back to a known state.
        // SAFETY: DMABUSMODE is a known-valid 32-bit register.
        unsafe { dma::set_bits(DMABUSMODE, bus_mode::SW_RESET) };

        // Compute in u64 then clamp to u32 so very large `timeout_ms`
        // values still produce a sane upper bound on the polling loop:
        // `timeout_ms * 1000` would wrap a u32 once `timeout_ms`
        // exceeds ~4.29 × 10⁶ ms (~71.6 minutes). Force at least one
        // iteration so a `timeout_ms = 0` caller still gets a single
        // read (the soft-reset bit usually clears within a few
        // microseconds — much faster than any meaningful timeout).
        let max_iters = (u64::from(self.timeout_ms) * 1000 / u64::from(RESET_POLL_INTERVAL_US))
            .clamp(1, u64::from(u32::MAX)) as u32;
        // Read-then-(maybe-)delay loop: after the last read we go
        // straight to `Timeout` instead of sleeping a final poll
        // interval we'd never check. This keeps the actual wait at
        // ≤ `(max_iters - 1) * RESET_POLL_INTERVAL_US` (matching
        // `timeout_ms`) and lets `timeout_ms = 0` actually mean
        // "single read, no sleep" once the iteration clamp gives us
        // exactly one pass.
        let mut iter = 0u32;
        loop {
            // SAFETY: same address, read-only volatile.
            let still_in_progress = unsafe { dma::read(DMABUSMODE) } & bus_mode::SW_RESET != 0;
            if !still_in_progress {
                return Ok(());
            }
            iter += 1;
            if iter >= max_iters {
                return Err(ResetError::Timeout);
            }
            self.delay.delay_us(RESET_POLL_INTERVAL_US);
        }
    }

    /// Configured timeout in milliseconds.
    pub fn timeout_ms(&self) -> u32 {
        self.timeout_ms
    }
}

#[cfg(feature = "async")]
pub mod async_impl {
    //! Async-flavoured variant of [`super::ResetController`].
    //!
    //! Identical bit-poll semantics, but each poll-step `.await`s the
    //! delay so the calling task yields control to the executor instead
    //! of busy-waiting. Enabled via the `async` cargo feature.

    use embedded_hal_async::delay::DelayNs;

    use super::{ResetError, RESET_POLL_INTERVAL_US, SOFT_RESET_TIMEOUT_MS};
    use crate::regs::dma::{self, bus_mode, DMABUSMODE};

    /// Async counterpart of [`super::ResetController`]. Owns an
    /// [`embedded_hal_async::delay::DelayNs`] implementation and exposes
    /// `soft_reset` as an `async fn`.
    pub struct AsyncResetController<D: DelayNs> {
        delay: D,
        timeout_ms: u32,
    }

    impl<D: DelayNs> AsyncResetController<D> {
        /// Build a controller with the [`SOFT_RESET_TIMEOUT_MS`] default.
        pub fn new(delay: D) -> Self {
            Self {
                delay,
                timeout_ms: SOFT_RESET_TIMEOUT_MS,
            }
        }

        /// Build a controller with a caller-chosen timeout.
        pub fn with_timeout(delay: D, timeout_ms: u32) -> Self {
            Self { delay, timeout_ms }
        }

        /// Issue the DMA software reset and `.await` the polling loop
        /// until `DMABUSMODE.SWR` self-clears. Returns
        /// [`ResetError::Timeout`] if the bit does not clear within
        /// `timeout_ms`.
        pub async fn soft_reset(&mut self) -> Result<(), ResetError> {
            // SAFETY: DMABUSMODE is a known-valid 32-bit register.
            unsafe { dma::set_bits(DMABUSMODE, bus_mode::SW_RESET) };

            // Floor `max_iters` to 1 so a `timeout_ms = 0` caller
            // still gets a single check — the soft-reset bit usually
            // clears within microseconds.
            let max_iters = (u64::from(self.timeout_ms) * 1000 / u64::from(RESET_POLL_INTERVAL_US))
                .clamp(1, u64::from(u32::MAX)) as u32;
            // Read-then-(maybe-)await loop: after the last read we
            // return `Timeout` directly instead of yielding for a
            // final poll interval we'd never check, so the actual
            // wait stays ≤ `(max_iters - 1) * RESET_POLL_INTERVAL_US`
            // and `timeout_ms = 0` means "single read, no yield".
            let mut iter = 0u32;
            loop {
                // SAFETY: same address, read-only volatile.
                let still_in_progress = unsafe { dma::read(DMABUSMODE) } & bus_mode::SW_RESET != 0;
                if !still_in_progress {
                    return Ok(());
                }
                iter += 1;
                if iter >= max_iters {
                    return Err(ResetError::Timeout);
                }
                self.delay.delay_us(RESET_POLL_INTERVAL_US).await;
            }
        }

        /// Configured timeout in milliseconds.
        pub fn timeout_ms(&self) -> u32 {
            self.timeout_ms
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        // Compile-only check: confirms the public API takes any
        // `embedded_hal_async::delay::DelayNs` impl and returns an
        // awaitable `Result`. Not a runtime test — the polling body
        // would need an executor and a real (or mocked) MMIO surface.
        #[allow(dead_code)]
        async fn _compile_check<D: DelayNs>(d: D) -> Result<(), ResetError> {
            let mut ctl = AsyncResetController::with_timeout(d, 100);
            ctl.soft_reset().await
        }
    }
}
