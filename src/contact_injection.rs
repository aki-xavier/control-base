// contact_injection.rs — what a decided force DOES to the machine. The steps are not the point; the
// order and the signs are. The gate, the clamp-before-dashpot order and the dashpot's sign are what a
// second copy gets wrong quietly.
//
// Why it is split from contact_law.rs: how much and what that does are separable, and separating them
// is what lets the source of a ground force be swapped without the rest of the contact model
// changing.

use control_math::mat::Mat;
use control_math::vec3::Vec3;

/// ContactForce is one component of one slot's contribution. Why the dashpot part is carried
/// separately: recording only the total is what makes a drift impossible to attribute to the sensor
/// or to the damper.
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactForce {
    pub applied: f64,
    pub damp: f64,
}

/// ContactInjection holds the numbers that turn a wrench readout into generalized force. Why the
/// filter's memory is not held here: it belongs to whoever owns the slot layout, and is passed in.
#[derive(Clone, Debug)]
pub struct ContactInjection {
    /// 0 injects the raw readout.
    pub tau: f64,
    /// Why this bound does not apply to a penalty or a solved constraint: those are bounded by the
    /// machine's own stiffness and equations of motion, not by what a report may say.
    pub f_max: f64,
    pub damp: f64,
    pub floor: f64,
    /// Why the dashpot can be normal-only: a tangential one answers a rolling contact's rotation
    /// with a horizontal push, which is friction by another name and is already carried elsewhere.
    pub normal_only: bool,
    pub normal: Vec3,
    /// Why zero is meaningful: it leaves the contact frictionless.
    pub fric: f64,
    pub mu: f64,
    /// Why the lever arm is the patch's own half-width: it is the largest lever arm the patch offers.
    /// Zero disables the torsional term.
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

    /// Why the bound makes this friction and not a damper: it is Coulomb's limit, so the force
    /// cannot keep growing with speed past `mu * |f_n|`.
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

    /// Why the moment goes through the angular rows and not the linear ones: it is a moment about
    /// the contact normal, bounded by `mu * |f_n| * r_tor`.
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

    /// Why the gate exists: without it the dashpot would act on every slot on every tick.
    pub fn live(&self, s: usize, smooth: &[f64]) -> bool {
        let mag2 = smooth[3 * s] * smooth[3 * s]
            + smooth[3 * s + 1] * smooth[3 * s + 1]
            + smooth[3 * s + 2] * smooth[3 * s + 2];
        mag2 >= self.floor * self.floor
    }

    /// Why the bound is stated once, here: the force a readout commands and the bound that readout
    /// is trusted within are the same mechanism's numbers.
    pub fn clamp(&self, f: f64) -> f64 {
        if f > self.f_max {
            self.f_max
        } else if f < -self.f_max {
            -self.f_max
        } else {
            f
        }
    }

    /// Why one row per point rather than one push per slot: which point of a patch carries how much
    /// IS what a centre of pressure names. The force itself is already decided, and already clamped
    /// where a clamp applies.
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
        // differs in the last bit, and that last bit is amplified into a different trajectory.
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
            // along the normal is clamped to zero at most (unclamped, the dashpot injects suction and
            // drags the machine down).
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
