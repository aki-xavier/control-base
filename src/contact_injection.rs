use control_math::mat::Mat;
use control_math::vec3::Vec3;

fn clamp_abs(v: f64, cap: f64) -> f64 {
    if v > cap {
        cap
    } else if v < -cap {
        -cap
    } else {
        v
    }
}

fn row_dot(j: &Mat, k: usize, vel: &[f64]) -> f64 {
    let mut s = 0.0;
    for c in 0..vel.len() {
        s += j.at(k, c) * vel[c];
    }
    s
}

fn normal_dot(j: &Mat, n: &Vec3, vel: &[f64]) -> f64 {
    let mut s = 0.0;
    for c in 0..vel.len() {
        s += (j.at(0, c) * n.x + j.at(1, c) * n.y + j.at(2, c) * n.z) * vel[c];
    }
    s
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactForce {
    pub applied: f64,
    pub damp: f64,
}

#[derive(Clone, Debug)]
pub struct ContactInjection {
    pub tau: f64,
    pub f_max: f64,
    pub damp: f64,
    pub floor: f64,
    pub normal_only: bool,
    pub normal: Vec3,
    pub fric: f64,
    pub mu: f64,
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

    pub fn friction_row(
        &self,
        k: usize,
        jl: &Mat,
        vel: &[f64],
        f_n: f64,
        tau_c: &mut [f64],
    ) -> ContactForce {
        let vt = row_dot(jl, k, vel);
        let ft = clamp_abs(-self.fric * vt, self.mu * f_n.abs());
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

    pub fn torsional_row(
        &self,
        ja: &Mat,
        vel: &[f64],
        n: Vec3,
        f_n: f64,
        tau_c: &mut [f64],
    ) -> f64 {
        let wn = normal_dot(ja, &n, vel);
        let mz = clamp_abs(-self.c_tor * wn, self.mu * f_n.abs() * self.r_tor);
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

    pub fn live(&self, s: usize, smooth: &[f64]) -> bool {
        let mag2 = smooth[3 * s] * smooth[3 * s]
            + smooth[3 * s + 1] * smooth[3 * s + 1]
            + smooth[3 * s + 2] * smooth[3 * s + 2];
        mag2 >= self.floor * self.floor
    }

    pub fn clamp(&self, f: f64) -> f64 {
        clamp_abs(f, self.f_max)
    }

    pub fn row_at(
        &self,
        f: f64,
        k: usize,
        jl: &Mat,
        vel: &[f64],
        tau_c: &mut [f64],
    ) -> ContactForce {
        let vc = if self.normal_only {
            normal_dot(jl, &self.normal, vel)
        } else {
            row_dot(jl, k, vel)
        };
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
