// contact_injection.rs — the soft-constraint contact model the plants share (biped soles, fingertips,
// arm link origins): low-pass the wrench, gate the non-touching slots, clamp each slot, add the
// contact-point dashpot and push it through that point's J^T. The gate, the clamp-before-dashpot order
// and the dashpot's sign are what a copy gets wrong quietly.
//
// This is the half that says what a force DOES. The half that says how much — which point of a slot's
// set carries what, from the world's report, this side's own penalty or a solved constraint — is
// contact_law.rs, and the two are a plant's whole contact model (see that file's header).

use control_math::mat::Mat;
use control_math::vec3::Vec3;

/// ContactForce is what ONE component of ONE slot contributes: the force J^T receives and the dashpot
/// part of it (negative against approach). The split is carried so a plant recording the injected action
/// can attribute a drift to the sensor or to its own damper.
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactForce {
    /// the force the generalized rows receive
    pub applied: f64,
    /// the contact-point dashpot term
    pub damp: f64,
}

/// ContactInjection is one plant's contact model: the numbers that turn a wrench readout into
/// generalized force, and the arithmetic that applies them. The filter's memory is the plant's own
/// fc_smooth buffer, indexed by that plant's slot layout.
#[derive(Clone, Debug)]
pub struct ContactInjection {
    /// sensor low-pass time constant; 0 injects the raw readout
    pub tau: f64,
    /// most one component of one POINT may inject [N]: the bound a report-commanded force is clamped to
    /// (see `clamp`), and the number a plant shares out over a slot's contact set. A force decided by a
    /// penalty or solved as a constraint is NOT bounded by it — those are bounded by the machine's own
    /// stiffness and equations of motion, and the laws that decide them say so (contact_law.rs).
    pub f_max: f64,
    /// contact-point dashpot [N.s/m]
    pub damp: f64,
    /// a slot whose smoothed force is below this is not touching [N]
    pub floor: f64,
    /// normal_only makes the dashpot act along the contact NORMAL instead of on all three components,
    /// with normal its direction (the ground's +z for a walking machine): a tangential dashpot answers
    /// a rolling foot's rotation with a horizontal push, which is friction by another name and is
    /// already the engine's Coulomb model.
    pub normal_only: bool,
    pub normal: Vec3,
    /// TANGENTIAL (friction) gain [N.s/m] and its Coulomb bound as a fraction of the normal force.
    /// Zero leaves the contact frictionless.
    pub fric: f64,
    pub mu: f64,
    /// TORSIONAL friction about the contact normal [N.m.s/rad] and the arm its Coulomb bound is
    /// taken over [m] — the sole's own half-width, the largest arm the patch offers. Zero disables it.
    pub c_tor: f64,
    pub r_tor: f64,
}

impl ContactInjection {
    pub fn new(tau: f64, f_max: f64, damp: f64, floor: f64) -> ContactInjection {
        ContactInjection {
            tau,
            f_max,
            damp,
            floor,
            normal_only: false,
            normal: Vec3::ZERO,
            fric: 0.0,
            mu: 0.0,
            c_tor: 0.0,
            r_tor: 0.0,
        }
    }

    /// friction_row applies world component k (a tangential one) of slot s: a viscous force against the
    /// contact point's velocity along that axis, bounded by mu * |f_n| — Coulomb's limit, which is what
    /// makes it friction rather than a damper.
    pub fn friction_row(
        &self,
        k: usize,
        jl: &Mat,
        vel: &[f64],
        f_n: f64,
        tau_c: &mut [f64],
    ) -> ContactForce {
        let mut vt = 0.0;
        for j in 0..vel.len() {
            vt += jl.at(k, j) * vel[j];
        }
        let mut ft = -self.fric * vt;
        let cap = self.mu * f_n.abs();
        if ft > cap {
            ft = cap;
        } else if ft < -cap {
            ft = -cap;
        }
        if ft != 0.0 {
            for j in 0..vel.len() {
                tau_c[j] += jl.at(k, j) * ft;
            }
        }
        ContactForce {
            applied: ft,
            damp: ft,
        }
    }

    /// torsional_row applies the contact's TORSIONAL friction: a moment about the contact normal
    /// opposing the foot's rate about it, bounded by mu * |f_n| * r_tor. ja is the node's 3 x nv ANGULAR
    /// Jacobian, n the contact normal; the moment goes through the angular rows, not the linear ones.
    pub fn torsional_row(
        &self,
        ja: &Mat,
        vel: &[f64],
        n: Vec3,
        f_n: f64,
        tau_c: &mut [f64],
    ) -> f64 {
        let mut wn = 0.0;
        for j in 0..vel.len() {
            let ax = ja.at(0, j);
            let ay = ja.at(1, j);
            let az = ja.at(2, j);
            wn += (ax * n.x + ay * n.y + az * n.z) * vel[j];
        }
        let mut mz = -self.c_tor * wn;
        let cap = self.mu * f_n.abs() * self.r_tor;
        if mz > cap {
            mz = cap;
        } else if mz < -cap {
            mz = -cap;
        }
        if mz != 0.0 {
            for j in 0..vel.len() {
                tau_c[j] += (ja.at(0, j) * n.x + ja.at(1, j) * n.y + ja.at(2, j) * n.z) * mz;
            }
        }
        mz
    }

    /// smooth_into advances the sensor low-pass one tick, allocating the buffer on the first call.
    pub fn smooth_into(&self, fc: &[f64], smooth: &mut Vec<f64>, dt: f64) {
        let mut alpha = if self.tau > 0.0 { dt / self.tau } else { 1.0 };
        if alpha > 1.0 {
            alpha = 1.0;
        }
        if smooth.len() != fc.len() {
            *smooth = vec![0.0; fc.len()];
        }
        for i in 0..fc.len() {
            smooth[i] += (fc[i] - smooth[i]) * alpha;
        }
    }

    /// live reports whether slot s is touching at all: without this gate the dashpot would act on every
    /// slot on every tick.
    pub fn live(&self, s: usize, smooth: &[f64]) -> bool {
        let mag2 = smooth[3 * s] * smooth[3 * s]
            + smooth[3 * s + 1] * smooth[3 * s + 1]
            + smooth[3 * s + 2] * smooth[3 * s + 2];
        mag2 >= self.floor * self.floor
    }

    /// clamp is the bound a REPORT-commanded component is injected under: the most one component of one
    /// point may carry [N]. It is stated here, once, because the force a law commands from a readout and
    /// the bound that readout is trusted within are the same mechanism's numbers — see contact_law's
    /// ReportContact, its only caller.
    pub fn clamp(&self, f: f64) -> f64 {
        if f > self.f_max {
            self.f_max
        } else if f < -self.f_max {
            -self.f_max
        } else {
            f
        }
    }

    /// row_at adds the contact-point dashpot along one Jacobian row and the J^T push of the sum, given
    /// the force already in hand. A sole touching the ground at SEVERAL points uses it — one row per
    /// point, each with its own force and Jacobian — because the centre of pressure a balance law
    /// commands is which point of the patch carries how much; the force itself comes from a law
    /// (contact_law.rs) and the clamp, where a law applies one, has already been applied.
    pub fn row_at(
        &self,
        f: f64,
        k: usize,
        jl: &Mat,
        vel: &[f64],
        tau_c: &mut [f64],
    ) -> ContactForce {
        let mut vc = 0.0;
        if self.normal_only {
            // the contact-point velocity ALONG THE NORMAL: damping belongs to the approach
            for j in 0..vel.len() {
                vc += (self.normal.x * jl.at(0, j)
                    + self.normal.y * jl.at(1, j)
                    + self.normal.z * jl.at(2, j))
                    * vel[j];
            }
        } else {
            for j in 0..vel.len() {
                vc += jl.at(k, j) * vel[j];
            }
        }
        // the per-component path keeps the ORIGINAL arithmetic shape (one subtraction with the product
        // inline) because the C compiler contracts it into an FMA: a separately computed dashpot
        // differs in the last bit, and a legged machine amplifies that into a different trajectory.
        let applied: f64;
        let d: f64;
        if self.normal_only {
            let nk = if k == 0 {
                self.normal.x
            } else if k == 1 {
                self.normal.y
            } else {
                self.normal.z
            };
            let mut d_n = -self.damp * vc * nk;
            let mut applied_n = f + d_n;
            // A CONTACT CANNOT PULL: the dashpot may cancel the spring, never invert it, so the force
            // along the normal is clamped to zero at most (unclamped, this plant injected -142 N of
            // suction and dragged the machine down).
            if applied_n * nk < 0.0 {
                applied_n = 0.0;
                d_n = -f;
            }
            applied = applied_n;
            d = d_n;
        } else {
            d = -self.damp * vc;
            applied = f - self.damp * vc;
        }
        if applied != 0.0 {
            for j in 0..vel.len() {
                tau_c[j] += jl.at(k, j) * applied;
            }
        }
        ContactForce { applied, damp: d }
    }
}
