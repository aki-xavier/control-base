# control-base — the control stack's product-neutral base

A project of its own: `simu` (its only consumer) depends on it as a sibling path
dependency, so neither the `Plant` contract nor the efference copy lives in
`simu`'s tree. MIT-licensed (see `LICENSE`).

Two modules, no engine, no model, no layer:

```text
plant      the Plant contract a controller programs against: ten queries, every
           one stated in control-math types (joint_positions / joint_velocities /
           mass_matrix / bias_torques / gravity_torques, compute_jacobian (3 x n)
           and compute_full_jacobian (6 x n), body_pose, and the per-link frames
           the arm's avoidance layer reads)
efference  the copy of what a layer COMMANDED against what its sensor read, and
           their difference: measured = commanded + residual. Dependency-free —
           it imports nothing at all.
```

It depends on one crate, [`control-math`](../control-math), for the `Mat`, `Vec3`
and `Quat` the contract is written in. It has no `build.rs` and needs no engine.

## Why it is shared rather than owned

The two control lines consume it in different directions:

- the **arm's loop** (`simu`'s `ga_pid/task_loop.rs`) programs against the `Plant`
  contract and keeps an efference copy per joint, in [N.m];
- the **walk** (`simu`'s `gait_runtime/mod.rs`) keeps one per leg, in [N].

Neither line owns it, so neither line's crate can hold it. A biped-side home would
make the arm's law depend on the walk; a law-side home would make the walk depend
on the arm's project. The arm-side crate `ga-…` cannot hold it either: that is the
dependency this crate exists to break, since a law crate may not depend on `simu`.

## The charter, and the check that keeps it

**Zero-dependency value types, and the interfaces that state a boundary.** The
dependency set is exactly `{ control-math }` and is not meant to grow.

`tests/charter.rs` makes that checkable rather than promised, the way `simu`'s own
`tests/engine_isolation.rs` does:

- every import is `control-math` or this crate's own;
- no source names an engine token (`eng_`, `extern "C"`, the SDK prefixes), a model
  type (`BodyTree`, `PgaFk`, …), a concrete plant (`CEnginePlant`, `BipedPlant`, …)
  or either line's law (`PlaneTaskLoop`, `StandingLoop`, `GaitRuntime`, …) **in
  code** — the prose may name them, and does, because these two files have to say
  where their implementors and consumers live;
- the `[dependencies]` table holds exactly one entry, and there is no `build.rs`.

## Provenance

Extracted from `simu`'s `src/`, where the two items had ended up on the wrong side
of the boundary:

- the `Plant` trait sat at the bottom of `plant/mod.rs`, below 240 lines of C ABI
  handles (`Engine` / `Scene` / `Robot` and their `Drop` impls). The contract and
  the engine sharing one file is exactly why this boundary could not be a crate
  before.
- `Efference` was a module at the crate root, 95 lines and zero imports, consumed
  by the arm's loop and the walk's runtime.

The shape of the split was measured rather than chosen: the GA-PID core's entire
external dependency surface was three edges — `control-math`, `Efference`, and the
`Plant` trait — of which two are these.

## Building

```sh
cargo test          # 4 tests: the efference arithmetic, and this crate's charter
cargo clippy --all-targets -- -D warnings
```

`simu`'s `Makefile` runs this suite as part of `make test` and `make lint`
(`--manifest-path ../control-base/Cargo.toml`).
