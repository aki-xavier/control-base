// control-base — the control stack's product-neutral base, as a project of its own.
//
// FIVE MODULES AND NOTHING ELSE:
//   adapt              the cerebellar layer's error-driven calibration: a per-parameter trim with its
//                      limits, its rate, the evidence it has accumulated and the trace of its recent
//                      values, over errors the CALLER measures and orients. It holds no model of the
//                      plant, imports nothing at all, and is here because BOTH machines read it —
//                      the walk drives its runtime with it and the arm's learned-feedforward probe
//                      is the other consumer.
//   plant              the Plant contract a controller programs against: ten queries, every one of
//                      them stated in control-math types. It is where it is because the contract's
//                      POINT is that the dynamics backend can be swapped without the control law
//                      changing, so the file cannot live beside an implementation.
//   efference          the copy of what a layer COMMANDED against what its sensor read, and their
//                      difference — `measured = commanded + residual`, the split a contact detector
//                      wants. Dependency-free: it imports nothing at all.
//   contact_law        where a plant's contact force COMES FROM, as a strategy: the world's report
//                      taken at its word, this side's own geometric penalty, the two raised together,
//                      or a complementarity solve of the non-penetration condition against the
//                      machine's own dynamics. Arithmetic on numbers a plant READ — from an engine,
//                      or from a model of its own — and the seam that lets the two be swapped without
//                      the plant changing.
//   contact_injection  what a decided force DOES: low-pass the wrench, gate the non-touching slots,
//                      clamp each slot, add the contact-point dashpot, and push the sum through that
//                      point's J^T. It names no engine, and it is here because more than one plant
//                      uses it: it states once what a copy gets wrong quietly (the gate, the
//                      clamp-before-dashpot order, the dashpot's sign).
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

pub mod adapt;
pub mod contact_injection;
pub mod contact_law;
pub mod efference;
pub mod plant;
