# control-base — the control stack's product-neutral base

A project of its own, and not a module inside anything that uses it: the `Plant`
contract, the efference copy, the contact model the plants share and the calibration
belong to the boundary, not to a side of it. MIT-licensed (see `LICENSE`).

Five modules, no engine, no model, no layer:

```text
plant              the Plant contract a controller programs against. Structure as
                   data first — PlantStructure: DOF, which coordinates are DRIVEN,
                   how many leading ones are the base pose, the distal contacts and
                   what each can supply, the bodies, and the frame the plant
                   presents for task control — then the numeric core, every term
                   stated in control-math types (joint_positions / joint_velocities
                   / mass_matrix / bias_torques / gravity_torques), then the task
                   mapping addressed BY NAME: frame_pose / frame_jacobian (3 x n) /
                   frame_full_jacobian (6 x n), and the per-body frames a whole-body
                   avoidance layer reads
efference          the copy of what a layer COMMANDED against what its sensor read,
                   and their difference: measured = commanded + residual.
                   Dependency-free — it imports nothing at all.
contact_injection  the soft-constraint contact model the plants share: low-pass the
                   wrench, gate the non-touching slots, clamp each slot, add the
                   contact-point dashpot, push the sum through that point's J^T.
                   Arithmetic on numbers a plant read; it names no engine, and it
                   states once what a copy gets wrong quietly (the gate, the
                   clamp-before-dashpot order, the dashpot's sign).
contact_law        where a plant's contact force COMES FROM, as a strategy: the
                   world's report taken at its word, this side's own geometric
                   penalty, the two raised together, or a complementarity solve of
                   the non-penetration condition against the machine's own dynamics.
                   Arithmetic on numbers a plant READ — from an engine, or from a
                   model of its own — and the seam that lets the two be swapped
                   without the plant changing.
adapt              a layer's error-driven calibration: a per-parameter trim with its
                   limits, its rate, the evidence it has accumulated and the trace of
                   its recent values, over errors the CALLER measures and orients. It
                   holds no model of the plant and imports nothing at all, so it sits
                   where no single user of it could own it.
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
  the five above;
- the interfaces really are stated in the sources the check reads — the `Plant`
  contract, the shared contact model and its laws, and the calibration — so the check
  cannot pass by looking at files that hold nothing;
- the `[dependencies]` table holds exactly one entry, and there is no `build.rs`.

## Building

```sh
cargo test
cargo clippy --all-targets -- -D warnings
```
