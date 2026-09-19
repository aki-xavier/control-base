// control-base — the control stack's product-neutral base, as a project of its own.
//
// TWO MODULES AND NOTHING ELSE:
//   plant      the Plant contract a controller programs against: ten queries, every one of them
//              stated in control-math types. It is where it is because the contract's POINT is that
//              the dynamics backend can be swapped without the control law changing, so the file
//              cannot live beside an implementation.
//   efference  the copy of what a layer COMMANDED against what its sensor read, and their
//              difference — `measured = commanded + residual`, the split a contact detector wants
//              (simu's CORTEX_PROGRAM.md #4). Dependency-free: it imports nothing at all.
//
// THE CHARTER, which is what keeps this crate thin: zero-dependency value types, and the interfaces
// that state a boundary. It names no engine, no model and no layer, and its dependency set is
// exactly `{ control-math }`.
//
// WHY IT IS SHARED RATHER THAN OWNED: the two control lines consume it in different directions.
// The arm's loop programs against the Plant contract and keeps an efference copy per joint
// (simu's ga_pid/task_loop.rs); the walk keeps one per leg (simu's gait_runtime/mod.rs). Neither
// line owns it, so neither line's crate can hold it: a BIPED-side home would make the arm's law
// depend on the walk, and a law-side home would make the walk depend on the arm's project.
//
// PROVENANCE. Extracted from the simu crate's `src/`: the Plant trait out of plant/mod.rs, where it
// sat below 240 lines of C ABI handles — the contract and the engine in one file is exactly why the
// boundary could not be a crate before; and Efference out of efference.rs. simu consumes this as a
// sibling path dependency (`{ path = "../control-base" }`).

pub mod efference;
pub mod plant;
