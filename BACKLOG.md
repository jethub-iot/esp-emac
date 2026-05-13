# Backlog

Deferred items tracked for future work. Each entry describes the
trigger that should cause it to be re-visited.

## Frame-filter helpers — full coverage

The `regs::mac` module currently exposes RMW helpers for two of the
seven significant bits of `GMACFF` (`set_promiscuous` for `PR` and
`set_disable_broadcast` for `DBF`). The remaining bits each have a
legitimate use case but no helper today:

- `HASH_UNICAST` (bit 1) — unicast frames are passed through the hash
  filter table instead of strict `ADDR0` match. Useful for multi-MAC
  sniffers that want to accept a sparse, programmable set of source
  MACs without surrendering all unicasts via promiscuous mode.
- `HASH_MULTICAST` (bit 2) — same idea for multicast.
- `DA_INVERSE` (bit 3) — flips the unicast match polarity (accept
  frames whose destination does *not* match `ADDR0`). Niche; mostly
  test gear.
- `PASS_ALL_MULTICAST` (bit 4) — accept every multicast frame without
  consulting the hash table. The common case for IGMP-snooping-free
  multicast workloads.
- `RECEIVE_ALL` (bit 31) — overrides every other filter bit, including
  `DBF`. The "true" sniffer mode.

**Trigger for picking this up**: any consumer that needs one of these
bits in production (current consumers — `embassy_net::EmacDriver`,
the netraw-l2-tester research firmware — do not). Adding all five at
once keeps the API surface symmetric and matches the
drivers-cover-full-hardware-capability convention.

The constants for all five bits already exist in
`regs::mac::frame_filter` (`mac.rs` lines ~96–106). The implementation
work is purely the wrappers + tests + rustdoc.

## Sticky-counter wrapper for `DMASTATUS.OVF` / `DMASTATUS.UNF`

`EmacInstrumentation` already folds the `DMAMISSEDFR` clear-on-read
quirk. The DMA-status `OVF` and `UNF` flags have similar transient
semantics that callers reading them via `regs::dma::status()` may
miss. Wrapping them in a sticky accumulator would let consumers ask
"have we ever seen an overrun?" without polling at line rate.

**Trigger**: a consumer that needs cumulative overrun / underrun
counts for diagnostic dashboards. The netraw research firmware uses
the `EmacInstrumentation` snapshot API; if the same fields appear in
the snapshot, the wrapper falls out naturally.
