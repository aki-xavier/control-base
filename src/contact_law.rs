use crate::contact_injection::ContactInjection;
use control_math::mat::Mat;

pub struct ContactSlot<'a> {
    pub inj: &'a ContactInjection,
    pub report: [f64; 3],
    pub live: bool,
    pub pen: &'a [f64],
    pub k: f64,
    pub own_set: bool,
}

pub struct ContactConstraint {
    pub jn: Vec<f64>,
    pub pen: f64,
}

pub struct ContactSystem<'a> {
    pub mass: &'a Mat,
    pub rhs: &'a [f64],
    pub vel: &'a [f64],
    pub dt: f64,
    pub candidates: &'a [ContactConstraint],
}

pub trait ContactLaw {
    fn decide_into(&self, slot: &ContactSlot, out: &mut Vec<[f64; 3]>) -> bool;
}

pub struct ReportContact;

impl ContactLaw for ReportContact {
    fn decide_into(&self, slot: &ContactSlot, out: &mut Vec<[f64; 3]>) -> bool {
        out.clear();
        if !slot.live {
            return false;
        }
        let n = slot.pen.len();
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

pub struct SolveContact {
    pub beta: f64,
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
    pub fn may_decide(&self, slot: &ContactSlot) -> bool {
        slot.own_set
    }

    pub fn solve_into(&self, sys: &ContactSystem, out: &mut Vec<f64>) {
        let np = sys.candidates.len();
        out.clear();
        out.resize(np, 0.0);
        let nv = sys.rhs.len();
        if np == 0 || nv == 0 {
            return;
        }
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
