# control-base — the control stack's product-neutral base

A project of its own, and not a module inside anything that uses it: the `Plant`
contract, the efference copy, the contact model the plants share and the calibration
both machines read belong to the boundary, not to a side of it. MIT-licensed (see
`LICENSE`).

Four modules, no engine, no model, no layer:

```text
plant              the Plant contract a controller programs against: ten queries,
                   every one stated in control-math types (joint_positions /
                   joint_velocities / mass_matrix / bias_torques / gravity_torques,
                   compute_jacobian (3 x n) and compute_full_jacobian (6 x n),
                   body_pose, and the per-link frames a whole-body avoidance layer
                   reads)
efference          the copy of what a layer COMMANDED against what its sensor read,
                   and their difference: measured = commanded + residual.
                   Dependency-free — it imports nothing at all.
contact_injection  the soft-constraint contact model the plants share: low-pass the
                   wrench, gate the non-touching slots, clamp each slot, add the
                   contact-point dashpot, push the sum through that point's J^T.
                   Arithmetic on numbers a plant read; it names no engine, and more
                   than one plant uses it, which is why it is stated here once.
adapt              the cerebellar layer's error-driven calibration: a per-parameter
                   trim with its limits, its rate, the evidence it has accumulated and
                   the trace of its recent values, over errors the CALLER measures and
                   orients. It holds no model of the plant and imports nothing at all;
                   the walk drives its runtime with it and the arm's learned-feedforward
                   probe is the other consumer, which is why it is here rather than
                   under either.
```

It depends on one crate, [`control-math`](../control-math), for the `Mat`, `Vec3`
and `Quat` the contract is written in. It has no `build.rs` and needs no engine.

## Where the boundary sits, and why it is a crate

Two rules decide what may be here, and both are about direction rather than content:

- **A contract is not stated beside an implementation.** A trait that shares a file
  with an engine is one that engine can reach into, and the point of the contract is
  that the dynamics backend can be swapped without the control law changing.
- **Nothing here may sit above anything that uses it.** A user of this crate is a
  controller, a law or a runtime; were the contract stated inside one of them, every
  other would have to depend on that one, and a law crate would end up depending on a
  simulator.

So the crate is defined by what it is allowed to name — its own value types, and the
one arithmetic crate below it — and not by who consumes it. Nothing in it is stated
in terms of a fact that lives outside the crate.

## The charter, and the check that keeps it

**Zero-dependency value types, and the interfaces that state a boundary.** The
dependency set is exactly `{ control-math }` and is not meant to grow.

`tests/charter.rs` makes that checkable rather than promised:

- every import is `control-math` or this crate's own, and the module list is exactly
  the three above;
- no source, in **code**, names an engine token or SDK prefix, a model-layer type, a
  concrete implementor of this crate's own contract, or a control law — the four
  lists in the test hold those words, and they are the only place this crate writes
  them down;
- the `[dependencies]` table holds exactly one entry, and there is no `build.rs`.

## Building

```sh
cargo test          # 4 tests: the efference arithmetic, and this crate's charter
cargo clippy --all-targets -- -D warnings
```
