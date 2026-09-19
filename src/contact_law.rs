// contact_law.rs — ContactLaw, WHERE a plant's contact force comes from, as a strategy: the world's own
// report taken at its word, this side's own geometric penalty against a ground it knows, the two raised
// together, or a complementarity solve of the non-penetration condition against the machine's own
// dynamics. A law DECIDES and nothing else: it reads numbers the plant has already read, writes the force
// it commands at each point of a slot's contact set, and never sees an engine, a handle, an FK pass or a
// slot layout.
//
// contact_injection.rs is the other half — what happens to a force once it is decided (the clamp, the
// contact-point dashpot, the friction and torsional rows, the J^T push) — and the two together ARE a
// plant's contact model: a law says how much, the injection says what that does to the machine. A plant
// that swaps its law swaps where its ground force comes from and nothing else.
//
// WHY IT IS HERE AND NOT IN A PLANT: the decision is the same for every machine, whatever its contact
// geometry — a patch sampled at several points, a single reported point, a declared set of link origins.
// The arithmetic (share the report over the set, raise it to the penetration, bound it) is one statement
// for all of them, and a plant keeping its own copy is a plant that gets the sharing order, the gate or
// the bound wrong quietly. What stays with a plant is its slot layout, its FK, its Jacobians, its clamp
// and its record of what it injected.

use crate::contact_injection::ContactInjection;
use control_math::mat::Mat;

/// ContactSlot is ONE slot as a contact law sees it: what the world reported there, the geometry this
/// side knows about the same slot, and the numbers that bound what may be commanded. Every field is a
/// number the plant has ALREADY computed, which is what keeps a law free of an engine, a model and a
/// frame.
pub struct ContactSlot<'a> {
    /// this slot's own injection: the plant's numbers SHARED OUT over the slot's contact set (a patch
    /// sampled at four points carries a quarter of the clamp, the dashpot and the friction gain each),
    /// and the very object the plant will inject the answer through. A law reads its `floor` and its
    /// `clamp` from here rather than being handed a second copy of the same number.
    pub inj: &'a ContactInjection,
    /// what the world reported at this slot, [fx fy fz] in the world frame: the plant's low-passed
    /// readout for the slot's row of its own filter buffer. Zero when it reports nothing.
    pub report: [f64; 3],
    /// whether the world's OWN gate says the slot touches at all — the plant's report tested against
    /// the injection's floor. The answer is handed over rather than recomputed because that test IS the
    /// shared mechanism's: a law that re-derived it would be a second statement of the gate.
    pub live: bool,
    /// how far inside the contact surface each point of this slot's set is [m], never negative, one
    /// entry per point in the plant's own order — the count IS the set, and the order IS the order the
    /// forces come back in. A slot whose contact is entirely the world's passes a single zero: one
    /// point, whose depth is not this side's geometry to measure.
    pub pen: &'a [f64],
    /// this side's own stiffness for the slot's contact [N/m]: what a penalty reads and what a gate
    /// against a floor is stated with. Zero leaves the slot's geometry to the world.
    pub k: f64,
    /// whether the slot's contact SET is this side's own — a declared set rather than the world's own
    /// report. The one predicate that says which scheme MAY decide the slot.
    pub own_set: bool,
}

/// ContactConstraint is one candidate of a SOLVED slot: the normal row of its point's 3 x nv Jacobian,
/// and how far inside the surface the point is. The plant computes both — the Jacobian is geometry no
/// law can see — and hands them over because a multiplier is only defined against the dynamics the
/// constraint acts on.
pub struct ContactConstraint {
    /// the normal row of the point's Jacobian, in the plant's own coordinate order: the row the
    /// normal force pushes through
    pub jn: Vec<f64>,
    /// the point's penetration along the normal [m]: positive INSIDE the surface, negative above it.
    /// The velocity bound a solved contact is stated on unfolds from this, so its sign carries the
    /// constraint.
    pub pen: f64,
}

/// ContactSystem is the assembled dynamics a solved contact acts on: the machine's effective mass and
/// its force balance WITHOUT the solved slots' own contribution, the velocities the constraint is
/// stated on, the step, and the candidates. A multiplier is only defined against the system it acts on,
/// which is why a plant cannot be asked for a solved force before it has built this.
pub struct ContactSystem<'a> {
    /// the effective mass the multipliers act through (M with the step's implicit damping already on it)
    pub mass: &'a Mat,
    /// the force balance the solved slots' contact has NOT been added to
    pub rhs: &'a [f64],
    /// the generalized velocities, in the same coordinate order as `rhs`
    pub vel: &'a [f64],
    /// the step the multiplier is an impulse over: a force is the multiplier divided by it
    pub dt: f64,
    /// the candidates, in the plant's own order: the plant reads the answer in that order
    pub candidates: &'a [ContactConstraint],
}

/// ContactLaw is where a plant's contact force comes from. One call per slot per step decides it, and
/// the plant injects what comes back; a scheme whose force cannot be decided from the state alone (a
/// constraint's multiplier — see SolveContact) is stated ON a law rather than as one.
pub trait ContactLaw {
    /// decide_into decides slot `slot`: it fills `out` with the [fx fy fz] this law commands at each
    /// point of the slot's set, in the slot's own point order, and answers true; an untouched slot
    /// answers false with `out` cleared and costs the plant no Jacobian and no injection. `out` is the
    /// caller's buffer, reused across slots and steps, so a law in its steady state allocates nothing.
    fn decide_into(&self, slot: &ContactSlot, out: &mut Vec<[f64; 3]>) -> bool;
}

/// ReportContact is the world's report taken at its word: the readout for the slot, shared over the
/// whole set (one point carries all of it, a patch shares it evenly), each component clamped to what
/// the slot may command. It is the scheme for a rigid contact — a wall against a link, where what the
/// world reports IS the force at the point the plant injects it — and the honest default for a machine
/// with no ground model of its own.
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

/// PenaltyContact is THIS side's geometry and nothing else: `k * pen / n` at each point, the ground a
/// machine can run with when no report reaches it at all — a mirror that is not stepping, a scheme the
/// caller wants compared against the world's own. Because `k` is the stiffness of the whole patch,
/// dividing by the point count keeps a patch sampled at four points as stiff as the one point it is
/// calibrated against (`k` at each of four points is four times as stiff, and launches the machine).
///
/// It gates on its own geometry alone: a report under its floor is not evidence that the contact is off
/// the ground when the geometry says it is in it, which is the same reason the raised law exists.
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

/// FlooredContact is the report RAISED to this side's own geometry: the readout shared over the set by
/// penetration, and each point's normal never under its own penalty (`k * pen / n`). It is the scheme for
/// a machine whose contact set is declared rather than reported: the world's sensor stops reporting
/// exactly when a contact is visibly inside the floor, and the geometry cannot, so the larger of the two
/// is the force the ground is at least applying.
///
/// The clamp is deliberately NOT applied here. It is sized for what a report may SAY, while the two
/// quantities this law raises are already bounded by the plant: the report's own share never exceeds what
/// the slot may be commanded, and the penalty is the machine's own stiffness against a depth it
/// measured itself.
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

/// SolveContact is the complementarity scheme, stated ON a law rather than as one: the slots whose
/// contact set is this side's own have their normal force SOLVED against the machine's own dynamics
/// instead of commanded from the state. One constraint per point — `pen - dt * v_n+ >= 0`, `f >= 0`,
/// `f * (pen - dt * v_n+) = 0` — swept by projected Gauss-Seidel on the multipliers and pushed through
/// each point's own Jacobian. A multiplier is whatever makes the trajectory admissible, so an
/// asymmetry a law COMMANDS is not turned into LOAD the way a penalty's `k * pen` is (a penalty force
/// follows the geometry's asymmetry rather than making the trajectory admissible).
///
/// A solved slot declares its points and nothing else: no report, no penalty, no gate, no dashpot. Every
/// point of the set is a candidate — one far above the plane asks for a velocity bound it cannot
/// violate, so its multiplier comes out zero on its own — and what each carries is decided after the
/// solve, against the machine's own dynamics. A slot this side does NOT own keeps its law: a solved
/// contact is a statement about geometry this side declared, and a report is a force — the two cannot be
/// combined without inventing one.
pub struct SolveContact {
    /// the fraction of a point's EXISTING penetration the solve pushes out per step [0, 1]. A point that
    /// is not penetrating gets the exact one-step bound (`v_n+ >= pen/dt`), so a landing point stops AT
    /// the plane; 1.0 is the hard limit, and 0.2 is a conservative default.
    pub beta: f64,
    /// the Gauss-Seidel sweep count: accuracy rather than feasibility
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
    /// may_decide answers whether this scheme decides the slot: the constraints are the declared sets,
    /// and every other slot is the law's — the plant asks this BEFORE any force is commanded, so a solved
    /// slot costs a law nothing.
    pub fn may_decide(&self, slot: &ContactSlot) -> bool {
        slot.own_set
    }

    /// solve_into turns the step's candidates into their multipliers, one per candidate in the plant's
    /// own order: an impulse [N.s], whose quotient by the step is the force the plant pushes through the
    /// candidate's Jacobian. `out` is the caller's buffer.
    ///
    /// Every quantity here is built against `sys.mass` and `sys.rhs` — the system the multipliers act on
    /// — because the coupling between two points of one patch (this one does not sink, that one carries
    /// the load) lives in that matrix and nowhere else.
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
