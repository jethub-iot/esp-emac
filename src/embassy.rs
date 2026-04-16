// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Embassy-net driver integration for ESP32 EMAC (Phase 1 facade).
//!
//! The underlying implementation is provided by
//! [`ph_esp32_mac::integration::embassy_net`]. This module keeps the
//! esp-emac public surface (`EmacDriver`, `EmacDriverState`) with the
//! API the firmware uses (`new()` without args, `set_link_up/down()`,
//! `on_interrupt()`).

use core::task::Context;

use embassy_net_driver::{Capabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken};

use ph_esp32_mac::integration::embassy_net::{EmbassyEmac, EmbassyEmacState};

use crate::emac::Emac;

/// Shared state for the embassy-net EMAC driver.
///
/// Stores wakers for RX, TX, and link state notifications. Place in a
/// `static` so it is accessible from interrupt handlers and async tasks.
pub struct EmacDriverState {
    inner: EmbassyEmacState,
}

impl Default for EmacDriverState {
    fn default() -> Self {
        Self::new()
    }
}

impl EmacDriverState {
    /// Create a new driver state (link initially down).
    pub const fn new() -> Self {
        Self {
            inner: EmbassyEmacState::new(LinkState::Down),
        }
    }

    /// Get the current cached link state.
    pub fn link_state(&self) -> LinkState {
        self.inner.link_state()
    }

    /// Set the link state to up and wake waiters.
    pub fn set_link_up(&self) {
        self.inner.set_link_state(LinkState::Up);
    }

    /// Set the link state to down and wake waiters.
    pub fn set_link_down(&self) {
        self.inner.set_link_state(LinkState::Down);
    }

    /// Handle an EMAC interrupt — wake tasks and clear pending flags.
    ///
    /// Call this from the EMAC ISR.
    pub fn on_interrupt(&self) {
        self.inner.handle_interrupt();
    }

    /// Access the underlying ph-esp32-mac state (facade escape hatch).
    #[doc(hidden)]
    pub fn inner(&self) -> &EmbassyEmacState {
        &self.inner
    }
}

/// Receive token — thin re-export of ph-esp32-mac's token.
pub type EmacRxToken<'d, const RX: usize, const TX: usize, const BUF: usize> =
    ph_esp32_mac::integration::embassy_net::EmbassyRxToken<'d, RX, TX, BUF>;

/// Transmit token — thin re-export of ph-esp32-mac's token.
pub type EmacTxToken<'d, const RX: usize, const TX: usize, const BUF: usize> =
    ph_esp32_mac::integration::embassy_net::EmbassyTxToken<'d, RX, TX, BUF>;

/// Embassy-net driver for the ESP32 EMAC.
///
/// Wraps [`ph_esp32_mac::integration::embassy_net::EmbassyEmac`] with the
/// esp-emac public API surface.
pub struct EmacDriver<'d, const RX: usize, const TX: usize, const BUF: usize> {
    inner: EmbassyEmac<'d, RX, TX, BUF>,
    state: &'d EmacDriverState,
}

impl<'d, const RX: usize, const TX: usize, const BUF: usize> EmacDriver<'d, RX, TX, BUF> {
    /// Create a new embassy-net driver.
    pub fn new(emac: &'d mut Emac<RX, TX, BUF>, state: &'d EmacDriverState) -> Self {
        let inner = EmbassyEmac::new(emac.inner_mut(), state.inner());
        Self { inner, state }
    }

    /// Get a reference to the shared driver state.
    pub fn state(&self) -> &EmacDriverState {
        self.state
    }
}

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
        self.inner.receive(cx)
    }

    fn transmit(&mut self, cx: &mut Context<'_>) -> Option<Self::TxToken<'_>> {
        self.inner.transmit(cx)
    }

    fn link_state(&mut self, cx: &mut Context<'_>) -> LinkState {
        self.inner.link_state(cx)
    }

    fn capabilities(&self) -> Capabilities {
        self.inner.capabilities()
    }

    fn hardware_address(&self) -> HardwareAddress {
        self.inner.hardware_address()
    }
}

// Silence unused-import warnings when RxToken/TxToken traits are not
// directly called here (they are referenced through Driver impl).
const _: fn() = || {
    fn _check<T: RxToken>(_: &T) {}
    fn _check2<T: TxToken>(_: &T) {}
};

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

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
        static STATE: EmacDriverState = EmacDriverState::new();
        assert!(STATE.link_state() == LinkState::Down);
    }
}
