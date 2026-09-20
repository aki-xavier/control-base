// contact_law.rs — where a contact force COMES FROM, as a strategy. Why the decision lives here and
// not with the geometry that supplies its inputs: the arithmetic is the same whatever the contact
// geometry, so a second copy is a place to get the sharing order, the gate or the bound wrong
// quietly. What a law may see is therefore only numbers already computed — no engine, no handle, no
// FK pass.
//
// Why it is split from contact_injection.rs: deciding how much and applying what that does are
// separable, and separating them is what lets the source of a ground force be swapped without the
// rest of the contact model changing.

use crate::contact_injection::ContactInjection;
use control_math::mat::Mat;

/// ContactSlot is one slot as a law sees it. Why every field is a number already computed: that is
/// what keeps a law free of an engine, a model and a frame.
pub struct ContactSlot<'a> {
    /// Why a law reads `floor` and `clamp` from here instead of being handed a second copy: they are
    /// the same mechanism's numbers, and a second copy is a place to drift.
    pub inj: &'a ContactInjection,
    pub report: [f64; 3],
    /// Why the gate's answer is handed over rather than recomputed: that test IS the shared
    /// mechanism's, and re-deriving it would be a second statement of it.
    pub live: bool,
    /// One entry per point of the slot's set, in the order the forces come back in; the count IS the
    /// set. Why a world-supplied contact passes a single zero: its depth is not this side's geometry
    /// to measure.
    pub pen: &'a [f64],
    /// Why zero is meaningful: it leaves the slot's geometry to the world.
    pub k: f64,
    /// The one predicate that says which scheme MAY decide the slot.
    pub own_set: bool,
}

/// ContactConstraint is one candidate of a solved slot. Why the Jacobian row is handed over rather
/// than derived: it is geometry a law cannot see, and a multiplier is only defined against the
/// dynamics the constraint acts on.
pub struct ContactConstraint {
    pub jn: Vec<f64>,
    /// Positive INSIDE the surface, negative above it: the velocity bound a solved contact is stated
    /// on unfolds from this, so the sign carries the constraint.
    pub pen: f64,
}

/// ContactSystem is the assembled dynamics a solved contact acts on. Why the solve cannot happen
/// earlier: a multiplier is only defined against the system it acts on.
pub struct ContactSystem<'a> {
    pub mass: &'a Mat,
    /// The force balance the solved slots' own contact has NOT been added to.
    pub rhs: &'a [f64],
    pub vel: &'a [f64],
    pub dt: f64,
    /// The candidates, in the order the answer comes back in.
    pub candidates: &'a [ContactConstraint],
}

/// ContactLaw is where a contact force comes from. Why a scheme whose force cannot be decided from
/// the state alone — a constraint's multiplier — is stated ON this trait rather than as one of its
/// implementations: one call per slot per step has to answer with a force.
pub trait ContactLaw {
    /// Why an untouched slot must clear `out` and answer false: the buffer is reused across slots
    /// and steps, so a stale force is worse than none, and answering false is what lets a Jacobian
    /// and an injection be skipped. Why `out` is passed in: a law in its steady state allocates
    /// nothing.
    fn decide_into(&self, slot: &ContactSlot, out: &mut Vec<[f64; 3]>) -> bool;
}

/// ReportContact takes the world's report at its word. Why clamping is right here: the report IS the
/// force at the point it is injected, so only the trusted bound is left to apply. It is the honest
/// default for a machine with no ground model of its own.
pub struct ReportContact;

impl ContactLaw for ReportContact {
    fn decide_into(&self, slot: &ContactSlot, out: &mut Vec<[f64; 3]>) -> bool {
        out.clear();
        if !slot.live {
            return false;
        }
        let n = slot.pen.len();
        // one point carries the whole report; a set of them carries a share each, so the slot's total is
        // the readout either way
        let w = 1.0 / n as f64;
        for _ in 0..n {
            out.push([
                slot.inj.clamp(w * slot.report[0]),
                slot.inj.clamp(w * slot.report[1]),
                slot.inj.clamp(w * slot.report[2]),
            ]);
        }
        true
    }
}

/// PenaltyContact is this side's geometry and nothing else. Why dividing by the point count: `k` is
/// the stiffness of the whole patch, so `k` at each of four points would be four times as stiff and
/// launch the machine. Why it gates on its own geometry alone: a report under the floor is not
/// evidence that the contact is off the ground when the geometry says it is in it.
pub struct PenaltyContact;

impl ContactLaw for PenaltyContact {
    fn decide_into(&self, slot: &ContactSlot, out: &mut Vec<[f64; 3]>) -> bool {
        out.clear();
        let n = slot.pen.len();
        let mut sum_pen = 0.0;
        for p in slot.pen {
            sum_pen += *p;
        }
        if slot.k * sum_pen <= slot.inj.floor {
            return false;
        }
        for i in 0..n {
            out.push([0.0, 0.0, slot.k * slot.pen[i] / n as f64]);
        }
        true
    }
}

/// FlooredContact raises the report to this side's own geometry. Why it exists: a sensor stops
/// reporting exactly when a contact is visibly inside the floor, and the geometry cannot, so the
/// larger of the two is the force the ground is at least applying.
///
/// Why the clamp is deliberately NOT applied here: it is sized for what a report may SAY, while both
/// quantities this law raises are already bounded — the report's own share by the slot's command
/// bound, the penalty by the machine's own stiffness against a depth it measured itself.
pub struct FlooredContact;

impl ContactLaw for FlooredContact {
    fn decide_into(&self, slot: &ContactSlot, out: &mut Vec<[f64; 3]>) -> bool {
        out.clear();
        let n = slot.pen.len();
        let mut sum_pen = 0.0;
        for p in slot.pen {
            sum_pen += *p;
        }
        let f_geo = slot.k * sum_pen;
        if !slot.live && f_geo <= slot.inj.floor {
            return false;
        }
        for i in 0..n {
            // a patch whose points are all above the surface still carries the report, so the share a
            // point gets is even where there is no penetration to weight it by
            let w = if sum_pen > 0.0 {
                slot.pen[i] / sum_pen
            } else {
                1.0 / n as f64
            };
            let f_geo_i = slot.k * slot.pen[i] / n as f64;
            let mut f = [w * slot.report[0], w * slot.report[1], w * slot.report[2]];
            if f[2] < f_geo_i {
                f[2] = f_geo_i;
            }
            out.push(f);
        }
        true
    }
}

/// SolveContact solves the normal force against the machine's own dynamics instead of commanding it
/// from the state, for the slots whose contact set this side declares. Why it is not a penalty: a
/// multiplier is whatever makes the trajectory admissible, so an asymmetry in the geometry is not
/// turned into load the way `k * pen` follows it.
///
/// Why a slot this side does NOT own keeps its law: a solved contact is a statement about geometry
/// this side declared and a report is a force, and the two cannot be combined without inventing one.
/// Why every point of the set is a candidate: one far above the plane asks for a bound it cannot
/// violate, so its multiplier comes out zero on its own.
pub struct SolveContact {
    /// Why existing penetration is relaxed rather than exact: pushing a point out on the exact
    /// one-step bound would fling it at metres per second. 1.0 is the hard limit.
    pub beta: f64,
    /// Why the sweep count is accuracy and not feasibility: the projection is feasible from the
    /// first sweep.
    pub iters: usize,
}

impl Default for SolveContact {
    fn default() -> SolveContact {
        SolveContact {
            beta: 0.2,
            iters: 20,
        }
    }
}

impl SolveContact {
    /// Why it is asked before any force is commanded: a slot this scheme does not own then costs it
    /// nothing.
    pub fn may_decide(&self, slot: &ContactSlot) -> bool {
        slot.own_set
    }

    /// Why every quantity is built against `sys.mass` and `sys.rhs`: the coupling between two points
    /// of one patch — this one does not sink, that one carries the load — lives in that matrix and
    /// nowhere else. `out` holds impulses [N.s], one per candidate in the declared order; the step's
    /// quotient is the force.
    pub fn solve_into(&self, sys: &ContactSystem, out: &mut Vec<f64>) {
        let np = sys.candidates.len();
        out.clear();
        out.resize(np, 0.0);
        let nv = sys.rhs.len();
        if np == 0 || nv == 0 {
            return;
        }
        // a_nc is the acceleration the solved points do NOT feel, and d[p] the velocity one
        // newton-second of point p's normal impulse buys the whole machine
        let a_nc = sys.mass.solve(sys.rhs);
        let mut d: Vec<Vec<f64>> = Vec::with_capacity(np);
        let mut wpp = vec![0.0; np];
        let mut v0 = vec![0.0; np];
        for p in 0..np {
            let dp = sys.mass.solve(&sys.candidates[p].jn);
            let mut w = 0.0;
            for j in 0..nv {
                w += sys.candidates[p].jn[j] * dp[j];
            }
            wpp[p] = w;
            let mut v = 0.0;
            for j in 0..nv {
                v += sys.candidates[p].jn[j] * (sys.vel[j] + sys.dt * a_nc[j]);
            }
            v0[p] = v;
            d.push(dp);
        }
        // the LCP's own matrix: wmat[p][q] is the normal velocity point p gains from point q's
        // multiplier, which is the projection's coupling
        let mut wmat = vec![0.0; np * np];
        for p in 0..np {
            for q in 0..np {
                let mut s = 0.0;
                for j in 0..nv {
                    s += sys.candidates[p].jn[j] * d[q][j];
                }
                wmat[p * np + q] = s;
            }
        }
        // the velocity bound each point asks for: `pen/dt` is the exact one-step non-penetration bound,
        // NEGATIVE for a point above the plane, so a landing point is stopped AT the plane and a point
        // in the air never binds. Existing penetration is the only place a relaxation belongs: an exact
        // bound there would have to fling it out at metres per second.
        let mut b = vec![0.0; np];
        for p in 0..np {
            let pen = sys.candidates[p].pen;
            b[p] = if pen > 0.0 {
                self.beta * pen / sys.dt
            } else {
                pen / sys.dt
            };
        }
        for _it in 0..self.iters {
            for p in 0..np {
                if !wpp[p].is_finite() || wpp[p] <= 0.0 {
                    out[p] = 0.0;
                    continue;
                }
                let mut vp = v0[p];
                for q in 0..np {
                    vp += wmat[p * np + q] * out[q];
                }
                let next = out[p] + (b[p] - vp) / wpp[p];
                out[p] = if next > 0.0 { next } else { 0.0 };
            }
        }
    }
}
