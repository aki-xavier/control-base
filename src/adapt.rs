// adapt.rs — why the sign is not decided here: it cannot know which way a parameter's error responds,
// and guessing one would build in a machine's convention. So an error arrives already oriented —
// raising the trim must reduce it — and this is arithmetic over numbers handed in.

/// One tuned quantity, with the evidence a caller needs to tell a parameter still walking toward its
/// target from one already sitting at a bias — which a trim alone cannot say.
#[derive(Clone, Debug)]
pub struct AdaptParam {
    pub name: String,
    pub trim: f64,
    /// Why the limits exist: a calibration must not walk a gain into a regime never tested.
    pub lo: f64,
    pub hi: f64,
    pub rate: f64,
    /// Why the running mean is kept: with the last error it separates still-learning from
    /// converged-to-a-bias.
    pub last_err: f64,
    pub mean_err: f64,
    pub samples: usize,
    /// Why the trace holds the value from BEFORE the update: that is what makes a delayed error
    /// attachable to the value that caused it (see `learn_at`).
    pub trace: Vec<f64>,
    /// Why the spans are audited at all: a channel is fed once per its own event, and feeding it at
    /// the wrong period is a mistake that is otherwise silent. The audit refuses no sample; it makes
    /// the mistake readable.
    pub last_span: f64,
    pub mean_span: f64,
    pub suspect: usize,
}

/// SUSPECT_FRAC is how far below a channel's own mean a sample's span must fall to count as a
/// discipline violation: loose enough that a jittering interval never trips it, tight enough that
/// feeding a per-step channel at a tick's period always does.
pub const SUSPECT_FRAC: f64 = 0.25;
/// SUSPECT_WARMUP is how many samples a channel needs before its mean is trusted as a baseline: the
/// first samples ARE the mean, so a channel's opening samples cannot be judged against it.
pub const SUSPECT_WARMUP: usize = 4;

/// TRACE is how many of a parameter's own past values are kept. Why a bounded ring in samples and
/// not a log in seconds: the same 64 is a different horizon per channel, and the depth only has to
/// cover one step's delay.
pub const TRACE: usize = 64;

/// Adapt is pure arithmetic over numbers handed in: it reads no plant, no clock and no model, which
/// is what lets it be shared without being owned.
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

    /// Why a non-positive rate is accepted rather than rejected: it keeps a parameter recorded but
    /// untuned, frozen at its initial trim.
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

    /// Why a rate can be changed on a live parameter: freezing one for an A/B must keep the
    /// correction already earned and must not require un-registering it. Unknown names are no-ops.
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

    /// Why an unknown name answers 0 rather than panicking: a knob that was never registered means
    /// no correction, not a crash.
    pub fn trim(&self, name: &str) -> f64 {
        let Some(i) = self.index(name) else {
            return 0.0;
        };
        self.params[i].trim
    }

    /// Why the error must arrive oriented (raising the trim must reduce it): the sign is not
    /// knowable here, and guessing it would build in a machine's convention.
    pub fn learn(&mut self, name: &str, err: f64, dt: f64) -> f64 {
        let Some(i) = self.index(name) else {
            return 0.0;
        };
        self.steps += 1;
        let p = &mut self.params[i];
        p.last_err = err;
        p.samples += 1;
        // running mean, so convergence can be read without storing the history
        p.mean_err += (err - p.mean_err) / p.samples as f64;
        // the span audit: a non-positive or non-finite span is a violation on its face (a channel
        // handed `dt = 0` learns nothing); one far below its own mean is the per-tick-vs-per-step slip
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
        // the trace records the value this call is about to move, so `trace.last()` is the trim as
        // of the LAST call and `learn_at` can find the trim as of any call before it (see TRACE)
        p.trace.push(p.trim);
        if p.trace.len() > TRACE {
            p.trace.remove(0);
        }
        if p.rate > 0.0 && dt > 0.0 {
            p.trim += p.rate * err * dt;
            if p.trim > p.hi {
                p.trim = p.hi;
            } else if p.trim < p.lo {
                p.trim = p.lo;
            }
        }
        p.trim
    }

    /// Why the credit is computed at the OLD value and only the delta carried forward: the error was
    /// caused by the value the parameter had, not the one it has now. `age` counts in units of this
    /// parameter's own `dt`, the interval between the `learn` calls THIS parameter gets. An age
    /// inside one sample is therefore `learn` to the digit — not a second gain, not a slower rate.
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
            // the credit the error earns AT THE VALUE THAT CAUSED IT, clamped as `learn` clamps
            let mut earned = then + p.rate * err * dt;
            if earned > p.hi {
                earned = p.hi;
            } else if earned < p.lo {
                earned = p.lo;
            }
            // and the DIFFERENCE is what today's value moves by: the correction the past value would
            // have taken, carried forward to the value that inherited it
            p.trim += earned - then;
            if p.trim > p.hi {
                p.trim = p.hi;
            } else if p.trim < p.lo {
                p.trim = p.lo;
            }
        }
        p.trace.push(before);
        if p.trace.len() > TRACE {
            p.trace.remove(0);
        }
        p.trim
    }

    /// Why a second readout exists instead of a wider `report`: `report`'s text is pinned byte for
    /// byte by the suite, so the sampling discipline gets its own line.
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

    /// Why the rate is rendered in its shortest round-tripping form rather than through `g`: `g`
    /// renders a live 1e-4 as "0", the text of a frozen knob.
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
/// LeadTrim is a learned feedforward over a reference's own lead. Why only the SCALE is learned and
/// never the shape: the shape is geometry, and a badly learned scale is a worse lead where a badly
/// learned shape would be a wrong direction. The error must arrive oriented — `err_along > 0` means
/// behind its reference — for the same reason `Adapt` requires it.
#[derive(Clone, Debug)]
pub struct LeadTrim {
    pub ada: Adapt,
    /// Why the scale is capped besides the trim's own limits: a learner able to double a reference's
    /// velocity must be a decision, not an accident.
    pub max: f64,
}

impl LeadTrim {
    /// Why `max` bounds both directions symmetrically: the trim's limits are one interval, not two.
    pub fn new(rate: f64, max: f64) -> LeadTrim {
        let mut ada = Adapt::new();
        ada.add("lead", -max, max, rate);
        LeadTrim { ada, max }
    }

    pub fn observe(&mut self, err_along: f64, dt: f64) -> f64 {
        self.ada.learn("lead", err_along, dt)
    }

    /// Why an error can arrive late: the one read at a step is the outcome of the placement the
    /// PREVIOUS step's trim set, so `age` counts in this channel's own `dt` — `Adapt::learn_at`'s
    /// rule exactly.
    pub fn observe_at(&mut self, err_along: f64, dt: f64, age: f64) -> f64 {
        self.ada.learn_at("lead", err_along, age, dt)
    }

    /// Why 1.0 at a zero trim matters: that is the behaviour with no learning in it, bit for bit, so
    /// an A/B against a frozen channel compares the learning and not the wiring.
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
