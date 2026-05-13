//! Runtime observability snapshot of EMAC traffic and DMA state.
//!
//! Gated behind the `instrumentation` feature. Builds without this
//! feature pay zero — every counter, timestamp, and histogram bucket
//! is gated.
//!
//! # What this module exposes
//!
//! - [`EmacInstrumentation`] — snapshot type holding per-token counters,
//!   sticky DMA-error accumulators, and IRQ→token latency histograms.
//! - [`EmacInstrumentation::snapshot`] / [`EmacInstrumentation::reset`]
//!   — consumer-side API operating on
//!   [`crate::embassy_net::EmacDriverState`].
//! - [`HISTOGRAM_BUCKETS`], [`HISTOGRAM_UPPER_US`], [`histogram_bucket`]
//!   — bucketing helpers so host-side analysers can reproduce the same
//!   shape.
//!
//! The actual increment logic — ISR-side and `Driver` token-consume
//! paths — lives in [`crate::embassy_net`] where the counters are
//! defined as fields of [`EmacDriverState`]. That module re-uses the
//! `now_us` and `IRQ_TIMESTAMP_NONE` items below through `pub(crate)`
//! visibility.

use core::sync::atomic::Ordering;

use crate::embassy_net::EmacDriverState;

/// Number of buckets in the IRQ→token latency histograms.
///
/// Fixed at 16 so each [`EmacInstrumentation`] consumer can iterate
/// with a known shape and emit machine-readable output without dynamic
/// allocation. The boundaries are intentionally log-scaled — see
/// [`HISTOGRAM_UPPER_US`] for the exact thresholds.
pub const HISTOGRAM_BUCKETS: usize = 16;

/// Upper bound (microseconds, inclusive) for each [`HISTOGRAM_BUCKETS`]
/// bucket. A measurement `d_us` lands in the lowest-index bucket `i`
/// where `d_us <= HISTOGRAM_UPPER_US[i]`. The last entry is `u32::MAX`
/// so any measurement strictly greater than 50 ms still has a bucket
/// (any value with no smaller threshold lands in the overflow bucket).
///
/// Upper bounds (µs): `1, 2, 5, 10, 20, 50, 100, 200, 500, 1_000,
/// 2_000, 5_000, 10_000, 20_000, 50_000, u32::MAX` — fifteen explicit
/// log-scale thresholds plus an overflow bucket.
pub const HISTOGRAM_UPPER_US: [u32; HISTOGRAM_BUCKETS] = [
    1,
    2,
    5,
    10,
    20,
    50,
    100,
    200,
    500,
    1_000,
    2_000,
    5_000,
    10_000,
    20_000,
    50_000,
    u32::MAX,
];

/// Map a microsecond delta to a [`HISTOGRAM_BUCKETS`] index using the
/// thresholds in [`HISTOGRAM_UPPER_US`]. Pure function — exposed so
/// the same bucketing can be reproduced by host-side analysers.
#[inline]
#[must_use]
pub fn histogram_bucket(d_us: u32) -> usize {
    let mut i = 0;
    while i < HISTOGRAM_BUCKETS {
        if d_us <= HISTOGRAM_UPPER_US[i] {
            return i;
        }
        i += 1;
    }
    // Unreachable — the last bucket has `u32::MAX` as its upper bound —
    // but staying total avoids a panic in release-mode hardened builds.
    HISTOGRAM_BUCKETS - 1
}

/// Read the Xtensa LX6 monotonic microsecond clock.
///
/// Wraps [`esp_hal::time::Instant::now`] and returns the elapsed-since-
/// epoch value truncated to `u32`. The truncation wraps every ~71
/// minutes; instrumentation deltas are computed `now.wrapping_sub(prev)`
/// so the wrap is invisible as long as no single measurement spans
/// 71 minutes (typical measurement windows are bounded at tens of
/// seconds, so the budget is several orders of magnitude away).
#[inline]
pub(crate) fn now_us() -> u32 {
    let dur = esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_micros();
    // `esp_hal::time::Duration::as_micros` returns `u64` (note: this is
    // esp-hal's own Duration type, NOT `core::time::Duration` whose
    // `as_micros()` returns `u128`). The cast below is the explicit
    // truncation to the wrap-aware `u32` deltas the histograms use.
    dur as u32
}

/// Sentinel meaning "no RX IRQ timestamp recorded yet" — stored in
/// [`EmacDriverState::last_rx_irq_us`]. Chosen as `u32::MAX`; a real
/// `Instant::now()` returning `0xFFFF_FFFF` µs only happens at the
/// exact tick before wrap, so the false-negative rate is one sample
/// per ~71 minutes — well below the noise floor of the latency
/// histogram itself.
pub(crate) const IRQ_TIMESTAMP_NONE: u32 = u32::MAX;

/// Snapshot of every counter, byte total, and IRQ→token latency
/// histogram tracked by [`EmacDriverState`] under the `instrumentation`
/// feature. Reads non-atomically — each field is `Relaxed`-loaded
/// independently — so a snapshot taken concurrently with the EMAC ISR
/// can show slightly skewed totals. **Call from non-ISR context only.**
/// A seqlock or `portable-atomic`-backed `AtomicU64` would be required
/// to make the snapshot internally consistent against an ISR producer,
/// and the Xtensa LX6 has neither in hardware.
///
/// Byte counters (`rx_bytes` / `tx_bytes`) are raw byte totals in
/// `u32`. Xtensa LX6 has no `AtomicU64`, so the counter wraps every
/// 2³² bytes (≈ 4 GB; ≈ 340 s at sustained 100BASE-TX line rate).
/// Callers running measurement windows longer than that should
/// snapshot-and-reset periodically; for typical 30-second / 5-minute
/// windows the wrap is many orders of magnitude away.
///
/// Intended as an observability tool — counters for throughput
/// monitoring, histograms for diagnosing where latency is spent on
/// the IRQ → token → DMA path. Not part of the production embassy-net
/// data path.
///
/// # Single-observer convention
///
/// `snapshot` drains the clear-on-read `DMAMISSEDFR` hardware register
/// into the sticky `dma_missed_frames` / `dma_fifo_overflow`
/// accumulators with `fetch_add`. Two concurrent `snapshot()` calls
/// race on that read: one caller observes the full hardware delta
/// (and the accumulator advances correctly), the other observes zero
/// and its locally-computed `snapshot_after - snapshot_before` delta
/// is misleading. The sticky totals remain correct in either case,
/// so a single observer reading consecutive sticky values is fine —
/// but multi-observer setups need a `Mutex` around `snapshot()` /
/// `reset()` if per-call deltas matter.
#[derive(Debug, Clone, Copy, Default)]
pub struct EmacInstrumentation {
    /// Total invocations of `Driver::receive`.
    pub rx_calls: u32,
    /// `Driver::receive` invocations that returned a token pair.
    pub rx_some: u32,
    /// Total bytes received through the RX token's `consume`.
    /// Wraps every 2³² bytes — see the type-level docs.
    pub rx_bytes: u32,
    /// Frames dropped inside the RX token's `consume`.
    pub rx_dropped: u32,
    /// Total invocations of `Driver::transmit`.
    pub tx_calls: u32,
    /// `Driver::transmit` invocations that returned a token.
    pub tx_some: u32,
    /// Total bytes transmitted through the TX token's `consume`.
    /// Wraps every 2³² bytes — see the type-level docs.
    pub tx_bytes: u32,
    /// Frames dropped inside the TX token's `consume`.
    pub tx_dropped: u32,
    /// Sticky accumulator of the DMA Missed Frame Counter
    /// (`DMAMISSEDFR[15:0]`). The hardware register is clear-on-read,
    /// so [`EmacInstrumentation::snapshot`] reads it and rolls the
    /// delta into the [`EmacDriverState`] accumulator before returning
    /// the running total here.
    pub dma_missed_frames: u32,
    /// Sticky accumulator of the DMA FIFO Overflow Counter
    /// (`DMAMISSEDFR[31:16]`). Same clear-on-read semantics as
    /// [`Self::dma_missed_frames`].
    pub dma_fifo_overflow: u32,
    /// Distribution of RX latency from the EMAC IRQ that observed
    /// `RI` (rx_complete) to the moment the matching `RxToken`'s
    /// `consume` closure **returns** — i.e. it includes the user-
    /// provided closure body. Useful for measuring the full receiver
    /// path; not a pure scheduling-only metric. Bucket boundaries are
    /// given by [`HISTOGRAM_UPPER_US`].
    pub rx_irq_to_token_histogram: [u32; HISTOGRAM_BUCKETS],
    /// Distribution of `Emac::transmit` call latency (microseconds) —
    /// timestamped immediately before the engine push and immediately
    /// after it returns. **Does not** include the time the user closure
    /// spent preparing the frame, nor the wait between `Driver::transmit`
    /// returning a token and the caller calling `consume`. The bucket
    /// captures the EMAC-side latency only: internal `copy_from_slice`
    /// to the DMA buffer, descriptor arming, and `tx_poll_demand`.
    /// Bucket boundaries are given by [`HISTOGRAM_UPPER_US`].
    pub tx_token_to_dma_histogram: [u32; HISTOGRAM_BUCKETS],
}

impl EmacInstrumentation {
    /// Atomically (per-field) read every instrumentation counter on
    /// `state`, then read-and-accumulate the clear-on-read DMA
    /// missed-frame and FIFO-overflow counters. Subsequent reads see
    /// the running totals; the hardware register is zero after this
    /// call.
    ///
    /// # Precondition
    ///
    /// Performs a volatile MMIO read of `DMAMISSEDFR` via
    /// [`crate::regs::dma::missed_frames`]. The EMAC peripheral clock
    /// must be enabled before this is called — typically after
    /// `Emac::init`. Calling before the clock is on will bus-fault.
    ///
    /// Safe to call from any non-ISR context (Embassy task, blocking
    /// `main()`) once the precondition above is met. Mixed-precision
    /// tearing between fields is possible (see the type-level docs),
    /// but the worst outcome is a snapshot whose `rx_calls` and
    /// `rx_some` disagree by one or two — fine for ratio metrics.
    #[must_use]
    pub fn snapshot(state: &EmacDriverState) -> Self {
        // Roll the clear-on-read DMA counters into the sticky
        // accumulator first; `missed_frames()` clears the register, so
        // the running total in `state` is the only source of truth
        // going forward.
        let (mfc_delta, ovf_delta) = crate::regs::dma::missed_frames();
        state
            .dma_missed_frames
            .fetch_add(mfc_delta, Ordering::Relaxed);
        state
            .dma_fifo_overflow
            .fetch_add(ovf_delta, Ordering::Relaxed);

        let mut rx_hist = [0u32; HISTOGRAM_BUCKETS];
        let mut tx_hist = [0u32; HISTOGRAM_BUCKETS];
        for (i, b) in rx_hist.iter_mut().enumerate() {
            *b = state.rx_irq_to_token_us[i].load(Ordering::Relaxed);
        }
        for (i, b) in tx_hist.iter_mut().enumerate() {
            *b = state.tx_token_to_dma_us[i].load(Ordering::Relaxed);
        }

        Self {
            rx_calls: state.drv_rx_calls.load(Ordering::Relaxed),
            rx_some: state.drv_rx_some.load(Ordering::Relaxed),
            rx_bytes: state.drv_rx_bytes.load(Ordering::Relaxed),
            rx_dropped: state.drv_rx_dropped.load(Ordering::Relaxed),
            tx_calls: state.drv_tx_calls.load(Ordering::Relaxed),
            tx_some: state.drv_tx_some.load(Ordering::Relaxed),
            tx_bytes: state.drv_tx_bytes.load(Ordering::Relaxed),
            tx_dropped: state.drv_tx_dropped.load(Ordering::Relaxed),
            dma_missed_frames: state.dma_missed_frames.load(Ordering::Relaxed),
            dma_fifo_overflow: state.dma_fifo_overflow.load(Ordering::Relaxed),
            rx_irq_to_token_histogram: rx_hist,
            tx_token_to_dma_histogram: tx_hist,
        }
    }

    /// Zero every instrumentation counter on `state` and clear the
    /// DMA missed-frame / FIFO-overflow hardware register so the next
    /// [`EmacInstrumentation::snapshot`] starts from a clean baseline.
    ///
    /// # Precondition
    ///
    /// Like [`Self::snapshot`], performs a volatile MMIO read to drain
    /// the clear-on-read `DMAMISSEDFR` register. The EMAC peripheral
    /// clock must be enabled before this is called.
    ///
    /// Per-field stores are `Relaxed`; this is not synchronization but
    /// a "best-effort" reset between measurement windows. The ISR may
    /// continue to bump counters while `reset` runs — the worst case
    /// is that a few post-reset events land in the new window with
    /// pre-reset history attached, which is fine for the
    /// stage-comparison this API exists to support.
    pub fn reset(state: &EmacDriverState) {
        // Drain the clear-on-read hardware counter into the void —
        // we are about to zero the sticky accumulator anyway.
        let _ = crate::regs::dma::missed_frames();

        state.drv_rx_calls.store(0, Ordering::Relaxed);
        state.drv_rx_some.store(0, Ordering::Relaxed);
        state.drv_rx_bytes.store(0, Ordering::Relaxed);
        state.drv_rx_dropped.store(0, Ordering::Relaxed);
        state.drv_tx_calls.store(0, Ordering::Relaxed);
        state.drv_tx_some.store(0, Ordering::Relaxed);
        state.drv_tx_bytes.store(0, Ordering::Relaxed);
        state.drv_tx_dropped.store(0, Ordering::Relaxed);
        state.dma_missed_frames.store(0, Ordering::Relaxed);
        state.dma_fifo_overflow.store(0, Ordering::Relaxed);
        state
            .last_rx_irq_us
            .store(IRQ_TIMESTAMP_NONE, Ordering::Relaxed);
        for b in &state.rx_irq_to_token_us {
            b.store(0, Ordering::Relaxed);
        }
        for b in &state.tx_token_to_dma_us {
            b.store(0, Ordering::Relaxed);
        }
    }
}
