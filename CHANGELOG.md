# Changelog

All notable changes to `esp-emac` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed (breaking — module rename)

- Rename `esp_emac::embassy` module to `esp_emac::embassy_net` — the
  old name read as "embassy framework", but the module is in fact the
  embassy-net `Driver` impl. New name removes the ambiguity by matching
  the dependency name (`embassy-net`). Consumers must update their
  imports:

  ```rust
  // before
  use esp_emac::embassy::{EmacDefaultDriver, EmacDriverState};
  // after
  use esp_emac::embassy_net::{EmacDefaultDriver, EmacDriverState};
  ```

### Added

- `EmacInstrumentation` snapshot API for runtime observability of EMAC
  traffic and DMA state. Captures separate TX / RX call, byte and drop
  counters; 16-bucket log-scale IRQ→token and TX-token→DMA latency
  histograms; and sticky accumulators that fold the clear-on-read
  `DMAMISSEDFR.MFC` and `DMAMISSEDFR.OVF` counters so callers do not lose
  deltas between reads. Byte counters are raw `u32` (Xtensa LX6 has no
  `AtomicU64`); they wrap every 2³² bytes (≈ 4 GB ≈ 340 s at sustained
  100BASE-TX line rate) — callers running longer measurement windows
  should snapshot-and-reset periodically. `EmacInstrumentation::snapshot`
  and `EmacInstrumentation::reset` are safe from any non-ISR context
  (Embassy task, blocking `main()`) once the EMAC peripheral clock is
  enabled — both perform a volatile MMIO read of `DMAMISSEDFR` via
  `regs::dma::missed_frames()` and will bus-fault on hosts or before
  `Emac::init`. `reset` zeroes both the sticky counters and the
  underlying hardware register.
- `esp_emac::regs::dma::missed_frames()` returning `DMAMISSEDFR` as a decoded
  `(mfc, fifo_ovf)` pair. **Clear-on-read** — see the rustdoc for the
  consumer-side accounting requirements (`EmacInstrumentation` uses
  sticky accumulators to absorb this).
- `regs::mac::set_promiscuous(enable: bool)` toggling `GMACFF.PR` as a
  strict RMW so other filter bits survive untouched. Useful when MAC
  destination-address filtering would reject otherwise-wanted frames
  (e.g. loopback configurations where dst_mac == src_mac, or sniffer /
  monitor applications).
- `EmacBench = Emac<32, 16, 1600>` (with companion `BENCH_RX` / `BENCH_TX`
  consts) — deeper descriptor-ring configuration for high-pps
  workloads where the default 10/10/1600 ring drops frames. Memory
  footprint ≈ 76.5 KiB; caller is responsible for the budget. Not
  enabled by default — production firmware continues to use
  `EmacDefault` / `EmacSmall`.
- `embassy_net::EmacBenchDriver<'d>` — companion embassy-net driver alias
  matching the `EmacDefaultDriver` / `EmacSmallDriver` pattern, so
  callers can spell `EmacBenchDriver<'static>` instead of expanding
  `EmacDriver<'static, BENCH_RX, BENCH_TX, DEFAULT_BUF>` at every
  Embassy-net `Runner` site.
- New feature flag `instrumentation` (off by default; pulls in
  `embassy-net` + `esp-hal` for the `Instant::now()`-backed µs clock
  driving the histograms). Builds without this flag pay zero —
  every counter, timestamp, and histogram bucket is gated behind it.

### Notes

- No version bump in `Cargo.toml`. These additions will be exercised
  through consumer-side iterations before a coordinated minor release.

## [0.3.0] - 2026-05-08

### Added

- Hardware checksum offload for the EMAC: TX descriptors now request
  full TCP/UDP/ICMP + IPv4-header checksum insertion (`TDES0.CIC = 0b11`),
  and `GMACCONFIG.IPC` is set so the DMA verifies received checksums and
  silently drops bad frames before they reach the host. The embassy-net
  driver advertises `Checksum::None` for `ipv4`, `tcp`, `udp`, and
  `icmpv4` so smoltcp skips redundant software computation.

  Behavior is correct on either side: a peer talking to us sees normal
  Ethernet frames with valid checksums, and frames we receive with bad
  checksums never appear at the application. Two new unit tests cover
  the descriptor flag and the advertised capabilities.

  Verified on jxd-pm380-e1eth + LAN8720A. No measurable throughput
  delta on the iperf2 loopback (smoltcp packet pipeline is the
  dominant cost), but eliminates a real per-packet CPU expense and
  enables future zero-copy work.

## [0.2.0] - 2026-05-06

### Breaking

- `Speed` and `Duplex` are now re-exports of `eth_mdio_phy::{Speed,
  Duplex}` (gated by feature `mdio-phy`) rather than locally
  defined enums with `From<eth_mdio_phy::*>` conversions. Call
  sites that used to write `emac.set_speed(status.speed.into())`
  drop the `.into()` — the types are literally the same now.
  Without the `mdio-phy` feature the symbols are no longer exposed
  from the crate; drop down to
  `crate::regs::mac::set_speed_100mbps` /
  `set_duplex_full` for raw register control.
- `Emac::stop()` now returns `Err(EmacError::TxFlushTimeout)` when
  the FTF poll exhausts `TX_FIFO_FLUSH_TIMEOUT_US` instead of
  silently returning `Ok(())`. The rest of the teardown still runs
  unconditionally, so the driver state still ends up at
  `Initialized` and `start()` is safe to retry — the new error is
  a recoverable warning, not a state-machine corruption signal.
  Callers that pattern-match on `Ok(())` need a wildcard or
  explicit handling of the new variant.
- MSRV bumped from 1.75 to 1.88, matching what `esp-hal = "1.1"`
  declares in its own manifest (`esp-hal 1.0` carried the same
  1.88 pin). The previous declaration was mis-advertised.
- Bump `eth-mdio-phy` dependency pin from `^0.1.1` to `^0.2.0`.
  Trait crate's `Speed`/`Duplex` became `#[non_exhaustive]` in
  that release.
- Bump `embassy-sync` from `^0.7` to `^0.8` (cascade from
  `esp-hal 1.1` requirement).
- Bump `esp-hal` from `^1.0.0` to `^1.1.0`.

### Added

- `EmacError::TxFlushTimeout` variant (lands as a non-breaking
  variant addition because `EmacError` is `#[non_exhaustive]`).
- `Emac::set_speed` / `set_duplex` now match each `Speed`/`Duplex`
  variant explicitly. The trait-crate types became `#[non_exhaustive]`
  in `eth-mdio-phy 0.2`, so a future variant (e.g. a hypothetical
  `Speed::Mbps1000`) compiles transparently but has no register
  encoding on ESP32 EMAC. Such inputs are clamped to 100 Mbps
  (highest mode the peripheral physically supports) / Full duplex,
  with a `defmt::warn!` under the `defmt` feature flagging the
  mismatch.

### Fixed

- Two clippy errors under `cargo +esp clippy --target
  xtensa-esp32-none-elf -D warnings`:
  `redundant_guards` at `emac.rs:208` (collapsed
  `RmiiClockConfig::InternalApll { gpio, .. } if matches!(gpio, ClkGpio::Gpio0)`
  into `RmiiClockConfig::InternalApll { gpio: ClkGpio::Gpio0, .. }`)
  and `let_unit_value` at `emac.rs:474`
  (`esp_hal::interrupt::enable` returns `()`, the prior
  `let _ =` was dead).

### Documentation

- New "Recovery from task respawn" section in the embassy-net
  module rustdoc covering the `static mut EMAC` re-borrow path:
  `init()` is one-shot, so a respawned task must call `stop()` +
  `start()` to bring the engine back up rather than silently
  ignoring `EmacError::AlreadyInitialized`.
- `[package.metadata.docs.rs]` now sets `default-target =
  "riscv32imc-unknown-none-elf"` and drops the `esp-hal` feature.
  docs.rs cannot satisfy `xtensa-esp32-none-elf` (rustc upstream
  has no xtensa target, and docs.rs does not carry the Microchip
  fork), so the previous metadata produced a "Documentation:
  failed" badge. Matches the convention every other esp-rs crate
  (esp-hal, esp-println, esp-backtrace, esp-storage, esp-radio)
  already uses.
- Drop the WIP shields.io badge from README and remove the
  `### Pre-publication` section now that the crate ships on
  crates.io as 0.2.0.

## [0.1.2] - 2026-05-04

### Added

- `pub const DEFAULT_RX = 10`, `DEFAULT_TX = 10`, `DEFAULT_BUF = 1600`,
  `SMALL_RX = 4`, `SMALL_TX = 4` in `src/emac.rs` — single source of
  truth for the canonical ring sizings.
- `EmacDefault` / `EmacSmall` (MAC side) and the new
  `embassy::EmacDefaultDriver<'d>` / `embassy::EmacSmallDriver<'d>`
  driver-side aliases are all expressed in terms of those constants,
  so retuning a sizing updates all four aliases together. The driver
  aliases collapse `embassy_executor::task` signatures from
  `Runner<'static, EmacDriver<'static, 10, 10, 1600>>` to
  `Runner<'static, EmacDefaultDriver<'static>>`.

### Documentation

- Driver construction sites use `EmacDefaultDriver::new(emac, &state)`
  (the type alias's inherent `new`) instead of `EmacDriver::new(...)`,
  so the call site doesn't repeat the const generics.
- New "Lifetime alignment" section in `src/embassy.rs` accurately
  describing the constraints: `Emac` is a hardware-peripheral
  singleton (one EMAC on the ESP32, global MMIO), but
  `EmacDriverState` is **not** strictly singleton — multiple
  instances are fine for tests / sequential re-init; the constraint
  is that the ISR-bound instance must match the one passed to
  `EmacDriver::new`. Replaces the previous, overstated "per-process
  singleton" wording.
- Recommend `static mut EMAC: EmacDefault = EmacDefault::new(...)` +
  `unsafe { &mut *core::ptr::addr_of_mut!(EMAC) }` as the canonical
  storage pattern. `Emac::new` is `const fn`, so the value lives in
  BSS — no runtime stack involvement on boot. A
  `StaticCell::init(EmacDefault::new(...))` wrapper would risk
  materialising the ~32 KiB `EmacDefault` on the calling task's
  stack frame before moving it into the cell, which is enough to
  overflow tight ESP32 task stacks. Documentation in README,
  `src/lib.rs`, `src/embassy.rs`, and `examples/embassy_net_lan8720a.rs`
  all use this pattern with an explanatory comment.

### Notes

- `Emac::start()` and `Emac::stop()` are already idempotent (returning
  `Ok(())` on the matching state) — no changes needed; this changelog
  entry exists to make that contract visible after the migration §5.4
  audit.

## [0.1.1] - 2026-05-03

### Documentation

- Rewrite `README.md` as an integration guide: installation, feature
  matrix, compatibility table, full embassy-net + LAN8720A example,
  ISR binding recipe, troubleshooting checklist.
- Expand crate-level rustdoc in `src/lib.rs` to mirror the README on
  docs.rs.
- Add runnable example `examples/embassy_net_lan8720a.rs` (build with
  `--features esp-hal,mdio-phy,embassy-net --target xtensa-esp32-none-elf`).
- Add `documentation` and `readme` fields to `Cargo.toml`.

### Internal

- Move bare-metal example dependencies under
  `[target.'cfg(target_os = "none")'.dev-dependencies]` so `cargo test`
  on the host stays unaffected.

## [0.1.0] - 2026-04-29

### Added

- Initial public release.
- `Emac<RX, TX, BUF>` driver with native MAC/DMA bring-up.
- `EmacDriver` adaptor for `embassy-net` (feature `embassy-net`).
- `EspMdio` Station Management controller.
- `regs::{mac, dma, ext, gpio}` typed register helpers.
- `reset::ResetController` (blocking) and
  `reset::async_impl::AsyncResetController` (feature `async`).
- `clock` module — APLL 50 MHz programming and GPIO0/16/17 routing.

[0.2.0]: https://github.com/jethub-iot/esp-emac-rs/compare/v0.1.2...v0.2.0
[0.1.2]: https://github.com/jethub-iot/esp-emac-rs/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/jethub-iot/esp-emac-rs/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/jethub-iot/esp-emac-rs/releases/tag/v0.1.0
