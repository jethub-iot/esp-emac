# Changelog

All notable changes to `esp-emac` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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

- README quick-start, `src/lib.rs` crate-level rustdoc, `src/embassy.rs`
  module-level rustdoc, and `examples/embassy_net_lan8720a.rs` all
  switched from `static mut EMAC: Emac<...> = Emac::new(...)` +
  `unsafe { &mut *core::ptr::addr_of_mut!(EMAC) }` to the safer
  `static EMAC: StaticCell<EmacDefault> = StaticCell::new();` +
  `EMAC.init(EmacDefault::new(...))` pattern. No `static mut` and no
  `unsafe` block in the user-facing example.
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

### Internal

- `static_cell = "2"` added to host dev-dependencies (`cfg(not(target_os
  = "none"))`) so `cargo test --doc --features embassy-net` resolves
  the `StaticCell` import in the lib.rs doctest. Pure-Rust crate, no
  transitive cost.

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

[0.1.2]: https://github.com/jethub-iot/esp-emac-rs/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/jethub-iot/esp-emac-rs/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/jethub-iot/esp-emac-rs/releases/tag/v0.1.0
