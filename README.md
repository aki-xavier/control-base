# control-base — the control stack's product-neutral base

The `Plant` contract, the efference copy, the contact model and the calibration belong to the
boundary, not to a side of it. MIT-licensed (see `LICENSE`).

Five modules, no engine and no model:

```text
plant              the Plant contract — separate, because a contract stated beside an
                   implementation is one that implementation can reach into
efference          the commanded/measured split — separate, because it reads no plant and
                   holds no model
contact_injection  what a decided contact force does — the place a second copy gets the
                   gate, the clamp-before-dashpot order and the dashpot's sign wrong
contact_law        where a contact force comes from — the seam that lets the source be
                   swapped without the rest changing
adapt              error-driven calibration — it reads no plant, so it cannot be owned by
                   anything that reads one
```

It depends on one crate, [`control-math`](../control-math), for the `Mat`, `Vec3` and `Quat` the
contract is written in. It has no `build.rs`: a base that needs an engine at build time is not a base.

## Why it is a crate, and not a module

Two rules decide what may be here, and both are about direction rather than content:

- **A contract is not stated beside an implementation.** A trait that shares a file with an engine
  is one that engine can reach into, which defeats the point of the contract.
- **A shared value type may not sit above its holders.** One held by more than one party cannot live
  under any of them, or the others end up depending on whichever one it landed in.

So the crate is defined by what it may name — its own value types, and the one arithmetic crate below
it — rather than by anything outside it.

## The charter, and the check that keeps it

**Zero-dependency value types, and the interfaces that state a boundary.** The dependency set is
exactly `{ control-math }`, and why it is checked rather than promised: a base that grows a dependency
stops being a base by one convenient `use`, never by a decision.

`tests/charter.rs` reads the sources as text and holds the crate to it: every import is
`control-math` or this crate's own, the module list is exactly the five above, the interfaces really
are stated in the sources that check reads, and the `[dependencies]` table holds exactly one entry
with no `build.rs`.

## Building

```sh
cargo test
cargo clippy --all-targets -- -D warnings
```
