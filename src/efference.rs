// efference.rs — Efference, the copy of what a layer COMMANDED (simu's CORTEX_PROGRAM.md #4): a
// sensory reading is the world's part plus the machine's own, so the copy splits it into
// `measured = commanded (my own doing) + residual (the world's)`, which is what a contact detector
// wants. It reads no plant and holds no model; it is not a filter, so a command DELAYED relative to
// its reading subtracts the wrong thing — the caller states what it commanded this tick.

/// Efference is what a layer commanded against what its sensors read, and the difference. The
/// comparison is unconditional — any gating belongs at the USE (the walk runtime's
/// `step_on_residual`) — and the arm's loop holds an instance of the same type: one channel per joint
/// in torque [N.m] (`PlaneTaskLoop::eff`).
#[derive(Clone, Debug, Default)]
pub struct Efference {
    /// what the layer commanded, one entry per channel: [N] on the legs' vertical forces, [N.m] on
    /// the arm's joints
    pub commanded: Vec<f64>,
    /// what the sensor read back, same channels and unit (the arm's comes from the plant's own step
    /// rather than from a load cell)
    pub measured: Vec<f64>,
    /// `measured - commanded`: the part of the reading that is NOT the machine's own doing
    pub residual: Vec<f64>,
    /// how many comparisons have been taken
    pub samples: usize,
    /// the channels' unit, for `report` — empty means the legs' "N"
    pub unit: String,
}

impl Efference {
    pub fn new() -> Efference {
        Efference::default()
    }

    /// observe takes one tick's pair for the whole channel set and answers the total absolute
    /// residual in the channels' own unit. It resizes to the caller's channel count on the first
    /// call, so a machine with a different number of legs needs no configuration.
    pub fn observe(&mut self, commanded: &[f64], measured: &[f64]) -> f64 {
        let n = commanded.len().min(measured.len());
        if self.commanded.len() != n {
            self.commanded = vec![0.0; n];
            self.measured = vec![0.0; n];
            self.residual = vec![0.0; n];
        }
        let mut total = 0.0;
        for i in 0..n {
            self.commanded[i] = commanded[i];
            self.measured[i] = measured[i];
            self.residual[i] = measured[i] - commanded[i];
            total += self.residual[i].abs();
        }
        self.samples += 1;
        total
    }

    /// residual is one channel's difference, in that channel's own unit: positive means the sensor
    /// read MORE than the machine was doing, i.e. the world supplied some of it.
    pub fn residual(&self, i: usize) -> f64 {
        if i < self.residual.len() {
            self.residual[i]
        } else {
            0.0
        }
    }

    /// external_share is that residual as a share of what was measured: 1.0 when the machine is
    /// doing nothing, 0.0 when the whole reading is its own doing. A `measured` near zero (and so a
    /// 0/0) is answered as 0.0 rather than a panic.
    pub fn external_share(&self, i: usize) -> f64 {
        if i < self.measured.len() && self.measured[i].abs() > 1e-9 {
            self.residual[i] / self.measured[i]
        } else {
            0.0
        }
    }

    /// report is the layer's state as one line, labelled with `unit` (empty, the default, means the
    /// legs' "N"), for the caller's readout.
    pub fn report(&self) -> String {
        let unit = if self.unit.is_empty() {
            "N"
        } else {
            &self.unit
        };
        let mut cmd = String::new();
        let mut res = String::new();
        for i in 0..self.commanded.len() {
            cmd.push_str(&format!("{:.0} ", self.commanded[i]));
            res.push_str(&format!("{:+.0} ", self.residual[i]));
        }
        format!(
            "efference: commanded [{}] {unit}, residual [{}] {unit}, {} samples",
            cmd.trim_end(),
            res.trim_end(),
            self.samples
        )
    }
}
