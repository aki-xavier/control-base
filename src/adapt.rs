fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    if v > hi {
        hi
    } else if v < lo {
        lo
    } else {
        v
    }
}

#[derive(Clone, Debug)]
pub struct AdaptParam {
    pub name: String,
    pub trim: f64,
    pub lo: f64,
    pub hi: f64,
    pub rate: f64,
    pub last_err: f64,
    pub mean_err: f64,
    pub samples: usize,
    pub trace: Vec<f64>,
    pub last_span: f64,
    pub mean_span: f64,
    pub suspect: usize,
}

pub const SUSPECT_FRAC: f64 = 0.25;
pub const SUSPECT_WARMUP: usize = 4;

pub const TRACE: usize = 64;

#[derive(Clone, Debug)]
pub struct Adapt {
    pub params: Vec<AdaptParam>,
    pub steps: usize,
}

impl Adapt {
    pub fn new() -> Adapt {
        Adapt {
            params: Vec::new(),
            steps: 0,
        }
    }

    pub fn add(&mut self, name: &str, lo: f64, hi: f64, rate: f64) {
        self.params.push(AdaptParam {
            name: name.to_string(),
            trim: 0.0,
            lo,
            hi,
            rate,
            last_err: 0.0,
            mean_err: 0.0,
            samples: 0,
            trace: Vec::new(),
            last_span: 0.0,
            mean_span: 0.0,
            suspect: 0,
        });
    }

    pub fn set_rate(&mut self, name: &str, rate: f64) {
        if let Some(i) = self.index(name) {
            self.params[i].rate = if rate > 0.0 { rate } else { 0.0 };
        }
    }

    pub fn index(&self, name: &str) -> Option<usize> {
        for (i, p) in self.params.iter().enumerate() {
            if p.name == name {
                return Some(i);
            }
        }
        None
    }

    pub fn trim(&self, name: &str) -> f64 {
        let Some(i) = self.index(name) else {
            return 0.0;
        };
        self.params[i].trim
    }

    pub fn learn(&mut self, name: &str, err: f64, dt: f64) -> f64 {
        let Some(i) = self.index(name) else {
            return 0.0;
        };
        self.steps += 1;
        let p = &mut self.params[i];
        p.last_err = err;
        p.samples += 1;
        p.mean_err += (err - p.mean_err) / p.samples as f64;
        let span_bad = !dt.is_finite() || dt <= 0.0;
        if span_bad {
            p.suspect += 1;
        } else {
            if p.samples > SUSPECT_WARMUP && dt < SUSPECT_FRAC * p.mean_span {
                p.suspect += 1;
            }
            p.mean_span += (dt - p.mean_span) / p.samples as f64;
        }
        p.last_span = dt;
        p.trace.push(p.trim);
        if p.trace.len() > TRACE {
            p.trace.remove(0);
        }
        if p.rate > 0.0 && dt > 0.0 {
            p.trim = clamp(p.trim + p.rate * err * dt, p.lo, p.hi);
        }
        p.trim
    }

    pub fn learn_at(&mut self, name: &str, err: f64, age: f64, dt: f64) -> f64 {
        let Some(i) = self.index(name) else {
            return 0.0;
        };
        self.steps += 1;
        let p = &mut self.params[i];
        p.last_err = err;
        p.samples += 1;
        p.mean_err += (err - p.mean_err) / p.samples as f64;
        let back = if dt > 0.0 {
            ((age / dt).round().max(0.0) as usize).min(TRACE.saturating_sub(1))
        } else {
            0
        };
        let then = if back == 0 {
            p.trim
        } else {
            let n = p.trace.len();
            if n >= back {
                p.trace[n - back]
            } else {
                p.trim
            }
        };
        let before = p.trim;
        if p.rate > 0.0 && dt > 0.0 {
            let earned = clamp(then + p.rate * err * dt, p.lo, p.hi);
            p.trim = clamp(p.trim + earned - then, p.lo, p.hi);
        }
        p.trace.push(before);
        if p.trace.len() > TRACE {
            p.trace.remove(0);
        }
        p.trim
    }

    pub fn span_report(&self) -> Vec<String> {
        self.params
            .iter()
            .map(|p| {
                format!(
                    "{}: last_span={:.6} mean_span={:.6} n={} suspect={}",
                    p.name, p.last_span, p.mean_span, p.samples, p.suspect
                )
            })
            .collect()
    }

    pub fn report(&self) -> Vec<String> {
        let mut out = Vec::with_capacity(self.params.len());
        for p in &self.params {
            out.push(format!(
                "{}: trim={:.4} in [{:.3} {:.3}] rate={} err={:.4} mean={:.4} n={}",
                p.name, p.trim, p.lo, p.hi, p.rate, p.last_err, p.mean_err, p.samples
            ));
        }
        out
    }
}
#[derive(Clone, Debug)]
pub struct LeadTrim {
    pub ada: Adapt,
    pub max: f64,
}

impl LeadTrim {
    pub fn new(rate: f64, max: f64) -> LeadTrim {
        let mut ada = Adapt::new();
        ada.add("lead", -max, max, rate);
        LeadTrim { ada, max }
    }

    pub fn observe(&mut self, err_along: f64, dt: f64) -> f64 {
        self.ada.learn("lead", err_along, dt)
    }

    pub fn observe_at(&mut self, err_along: f64, dt: f64, age: f64) -> f64 {
        self.ada.learn_at("lead", err_along, age, dt)
    }

    pub fn scale(&self) -> f64 {
        1.0 + self.ada.trim("lead")
    }
}

impl Default for LeadTrim {
    fn default() -> Self {
        LeadTrim::new(0.0, 1.0)
    }
}

impl Default for Adapt {
    fn default() -> Self {
        Self::new()
    }
}
