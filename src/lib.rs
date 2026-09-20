// control-base — the control stack's product-neutral base. Why it is a crate of its own: a contract
// stated beside an implementation is one that implementation can reach into, and a value type held
// by more than one party cannot live under any one of them. So it is defined by what it may name —
// its own types, and the arithmetic crate below it — and its five modules are the whole of it.
//
//   plant              the contract; separate, because a contract beside an implementation is one
//                      that implementation can reach into
//   efference          the commanded/measured split; separate, because it reads no plant
//   contact_injection  what a decided contact force does; where a second copy gets the gate, the
//                      clamp-before-dashpot order and the dashpot's sign wrong
//   contact_law        where a contact force comes from; the seam that lets the source be swapped
//                      without the rest changing
//   adapt              error-driven calibration; it reads no plant, so it cannot be owned by
//                      anything that reads one
//
// THE CHARTER, which keeps the crate thin: zero-dependency value types, and the interfaces that state
// a boundary. The dependency set is exactly `{ control-math }`, and `tests/charter.rs` is what makes
// that a check rather than a promise.

pub mod adapt;
pub mod contact_injection;
pub mod contact_law;
pub mod efference;
pub mod plant;
