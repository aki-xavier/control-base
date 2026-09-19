// control-base — the control stack's product-neutral base, as a project of its own.
//
// TWO MODULES AND NOTHING ELSE:
//   plant      the Plant contract a controller programs against: ten queries, every one of them
//              stated in control-math types. It is where it is because the contract's POINT is that
//              the dynamics backend can be swapped without the control law changing, so the file
//              cannot live beside an implementation.
//   efference  the copy of what a layer COMMANDED against what its sensor read, and their
//              difference — `measured = commanded + residual`, the split a contact detector wants.
//              Dependency-free: it imports nothing at all.
//
// THE CHARTER, which is what keeps this crate thin: zero-dependency value types, and the interfaces
// that state a boundary. It names no engine, no model and no layer, and its dependency set is
// exactly `{ control-math }`.
//
// WHY IT IS A PROJECT OF ITS OWN: a user of this crate is a controller, a law or a runtime, and the
// two rules point the same way — a contract may not be stated beside an implementation, and a value
// type shared by more than one user may not live under any one of them. Were the contract stated
// inside one, every other would have to depend on it. So this crate is defined by what it is allowed
// to name — its own types, and the arithmetic crate below it — and not by who consumes it.

pub mod efference;
pub mod plant;
