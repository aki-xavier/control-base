// efference.rs — why the comparison is unconditional and why the command must be this tick's: the
// split `measured = commanded + residual` is meaningful only for one instant, so a command delayed
// against its reading subtracts the wrong thing. Gating is a policy, not arithmetic.

/// Efference holds the commanded/measured pair and their difference. The comparison is
/// unconditional: which readings matter is a question the arithmetic cannot answer.
#[derive(Clone, Debug, Default)]
pub struct Efference {
    pub commanded: Vec<f64>,
    pub measured: Vec<f64>,
    /// `measured - commanded`: the part of the reading that is NOT the machine's own doing.
    pub residual: Vec<f64>,
    pub samples: usize,
    /// Why empty is allowed: it prints as the default label.
    pub unit: String,
}

impl Efference {
    pub fn new() -> Efference {
        Efference::default()
    }

    /// Why it resizes rather than being configured: the channel count is data, not configuration.
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

    /// Positive means the reading was MORE than the machine was doing, i.e. the world supplied some
    /// of it.
    pub fn residual(&self, i: usize) -> f64 {
        if i < self.residual.len() {
            self.residual[i]
        } else {
            0.0
        }
    }

    /// Why a near-zero `measured` answers 0.0: the share would be a 0/0, and a division by zero is
    /// not a reading.
    pub fn external_share(&self, i: usize) -> f64 {
        if i < self.measured.len() && self.measured[i].abs() > 1e-9 {
            self.residual[i] / self.measured[i]
        } else {
            0.0
        }
    }

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
