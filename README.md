# control-base — the control stack's product-neutral base

The `Plant` contract, the efference copy, the contact model and the calibration belong to the
boundary, not to a side of it. MIT-licensed (see `LICENSE`).

Five modules, no engine and no model:

```text
plant              the Plant contract — separate, because a contract stated beside an
                   implementation is one that implementation can reach into — and the
                   pose -> motor conversion its poses are handed out in
efference          the commanded/measured split — separate, because it reads no plant and
                   holds no model
contact_injection  what a decided contact force does — the place a second copy gets the
                   gate, the clamp-before-dashpot order and the dashpot's sign wrong
contact_law        where a contact force comes from — the seam that lets the source be
                   swapped without the rest changing
adapt              error-driven calibration — it reads no plant, so it cannot be owned by
                   anything that reads one
```

It depends on two crates: [`control-math`](../control-math), for the `Mat`, `Vec3` and `Quat` the
contract's arithmetic is written in, and [`pga`](../pga), for the multivector the contract hands its
poses out in. `pga` is the zero-dependency crate of that shared value type, so naming it costs a
boundary statement and nothing else. It has no `build.rs`: a base that needs an engine at build time is
not a base.

## Why it is a crate, and not a module

Two rules decide what may be here, and both are about direction rather than content:

- **A contract is not stated beside an implementation.** A trait that shares a file with an engine
  is one that engine can reach into, which defeats the point of the contract.
- **A shared value type may not sit above its holders.** One held by more than one party cannot live
  under any of them, or the others end up depending on whichever one it landed in.

So the crate is defined by what it may name — its own value types, the one arithmetic crate below it,
and the algebra of the value type its poses are handed out in — rather than by anything outside it.

## The charter, and the check that keeps it

**Zero-dependency value types, and the interfaces that state a boundary.** The dependency set is
exactly `{ control-math, pga }`, and why it is checked rather than promised: a base that grows a
dependency stops being a base by one convenient `use`, never by a decision.

`pga` was added as a decision and not as a `use`: the contract states its poses as `(Vec3, Quat)`, and
a caller reading geometry through the algebra needs the same pose as one multiplicative element. It is
`pga` that holds that shared value type, and a value type held by more than one party lives in its own
crate — which is the second rule above, applied. Nothing that would need the model, a plant or an
engine came with it.

`tests/charter.rs` reads the sources as text and holds the crate to it: every import is `control-math`,
`pga` or this crate's own, the module list is exactly the five above, the interfaces (and the motor the
poses are handed out in) really are stated in the sources that check reads, and the `[dependencies]`
table holds exactly those two entries with no `build.rs`.

## Building

```sh
mbx test
mbx clippy --all-targets -- -D warnings
```
