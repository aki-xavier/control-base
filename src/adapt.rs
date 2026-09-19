// adapt.rs — Adapt, the cerebellar layer's error-driven calibration: a per-parameter trim (its
// limits, its rate, the evidence it has accumulated, and the trace of its own recent values) over
// errors the caller measures. It holds no model of the plant: the caller computes and orients every
// error so that RAISING the trim reduces it. The layer map is in AGENTS.md, pinned by tests/layering.rs.

/// AdaptParam is one tuned quantity: its trim, its limits, its rate, and the evidence of whether it
/// is still learning.
#[derive(Clone, Debug)]
pub struct AdaptParam {
    pub name: String,
    /// the trim the caller adds to its parameter's nominal value
    pub trim: f64,
    /// the trim's own limits: a calibration must not walk a gain into a regime never tested
    pub lo: f64,
    pub hi: f64,
    /// learning rate [1/s]: the trim moves by rate * err * dt
    pub rate: f64,
    /// the last error seen and the mean over the trim's life, so a caller can tell "still learning"
    /// from "converged to a bias"
    pub last_err: f64,
    pub mean_err: f64,
    pub samples: usize,
    /// the eligibility trace: this parameter's own recent values, newest LAST, one entry per `learn`
    /// call and the value it held BEFORE that call's update — what makes a DELAYED error attachable
    /// to the value that caused it (see `learn_at`)
    pub trace: Vec<f64>,
    /// the sampling discipline, audited: the span of the last sample the channel was handed [s], its
    /// mean, and how many samples arrived far below that mean. A channel is fed once per ITS OWN
    /// event with that event's interval; the audit refuses no sample, it makes the mistake readable
    pub last_span: f64,
    pub mean_span: f64,
    /// samples whose span was non-positive, non-finite, or below a quarter of this channel's own
    /// running mean (< `SUSPECT_FRAC`), counted after the mean has `SUSPECT_WARMUP` samples behind it
    pub suspect: usize,
}

/// SUSPECT_FRAC is how far below a channel's own mean a sample's span must fall to count as a
/// discipline violation: loose enough that a jittering interval never trips it, tight enough that the
/// per-tick-vs-per-step mistake (a ratio of ~1750 on the walk) always does.
pub const SUSPECT_FRAC: f64 = 0.25;
/// SUSPECT_WARMUP is how many samples a channel needs before its mean is trusted as a baseline: the
/// first samples ARE the mean, so a channel's opening samples cannot be judged against it.
pub const SUSPECT_WARMUP: usize = 4;

/// TRACE is how many of a parameter's own past values are kept: one per `learn` call, newest last, a
/// bounded ring rather than a log. Counted in samples and not seconds, so the same 64 is a different
/// horizon per channel (22.4 s per step, 12.8 ms per tick) and covers one step's delay.
pub const TRACE: usize = 64;

/// Adapt holds the layer's parameters: pure arithmetic over errors the caller
/// measures, reading no plant, a clock, or a model.
#[derive(Clone, Debug)]
pub struct Adapt {
    pub params: Vec<AdaptParam>,
    /// total error samples taken, for the caller's readout
    pub steps: usize,
}

impl Adapt {
    /// new builds an empty layer. Parameters are added by name, so a caller states what it is
    /// calibrating.
    pub fn new() -> Adapt {
        Adapt {
            params: Vec::new(),
            steps: 0,
        }
    }

    /// add registers a parameter. A non-positive rate leaves it frozen at its initial trim, which is
    /// how a caller keeps a knob recorded but untuned.
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

    /// set_rate changes a registered parameter's rate — how a caller freezes a LIVE channel for an A/B
    /// without un-registering it (rate 0 keeps the correction earned so far); unknown names are no-ops.
    pub fn set_rate(&mut self, name: &str, rate: f64) {
        if let Some(i) = self.index(name) {
            self.params[i].rate = if rate > 0.0 { rate } else { 0.0 };
        }
    }

    /// index finds a parameter's slot, or none.
    pub fn index(&self, name: &str) -> Option<usize> {
        for (i, p) in self.params.iter().enumerate() {
            if p.name == name {
                return Some(i);
            }
        }
        None
    }

    /// trim reads a parameter's current trim (0 for an unknown name, so a caller
    /// that forgets to register a knob gets "no correction" rather than a panic).
    pub fn trim(&self, name: &str) -> f64 {
        let Some(i) = self.index(name) else {
            return 0.0;
        };
        self.params[i].trim
    }

    /// learn feeds one error sample to a parameter and returns the trim to use: the integral of the
    /// error, clamped. The caller orients the error (RAISING the trim must reduce it); this layer
    /// cannot know that sign, and guessing it would build in a per-robot convention.
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

    /// learn_at is `learn` for an error that arrived late: `age` [s] counts in units of the caller's
    /// own `dt`, the interval between the `learn` calls THIS parameter gets (not the tick, when its
    /// error arrives once per step). The credit is computed against the trim the parameter HAD `age`
    /// seconds ago — the value that caused the error — and only the DELTA is carried to today's value,
    /// so an age inside one sample IS `learn` to the digit; it is not a second gain or a slower rate.
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
        // the value LIVE `back` samples ago: `back` 0 is the value now, one back the trace's newest entry
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

    /// report is the layer's state as one line per parameter: the trim, its bounds, the rate and the
    /// error evidence. The rate prints in its shortest round-tripping form rather than through `g`,
    /// which renders a live 1e-4 as "0" — the text of a frozen knob.
    /// span_report is the sampling discipline's readout: per channel, the span of its last sample, its
    /// mean span and how many samples arrived far below that mean (a second line, because `report`'s
    /// text is pinned byte for byte by `tests/adapt.rs`).
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
/// LeadTrim is the learned feedforward of this row's Phase 5 (`CEREBELLUM_PROGRAM.md`): a trim on a
/// reference's own lead (its velocity and acceleration terms) learned from the along-track error of a
/// repeated movement. Only the SCALE is learned, not the shape (`TaskSpec.lead` puts the shape through
/// J+) — a badly learned scale is a worse lead. The caller orients the error: `err_along > 0` must mean
/// "behind its reference", so raising the trim reduces it.
#[derive(Clone, Debug)]
pub struct LeadTrim {
    /// the underlying parameter: one trim, its limits and its rate, held by `Adapt`
    pub ada: Adapt,
    /// the trim's own cap on the SCALE, i.e. the largest multiplier the lead may carry (a learner that
    /// can double a reference's velocity must be a decision, not an accident)
    pub max: f64,
}

impl LeadTrim {
    /// new registers the one parameter. `rate` is in the same units as every other rate here
    /// (`trim += rate * err * dt`), and `max` bounds BOTH directions symmetrically.
    pub fn new(rate: f64, max: f64) -> LeadTrim {
        let mut ada = Adapt::new();
        ada.add("lead", -max, max, rate);
        LeadTrim { ada, max }
    }

    /// observe feeds one sample of the movement's own along-track error and answers the trim.
    pub fn observe(&mut self, err_along: f64, dt: f64) -> f64 {
        self.ada.learn("lead", err_along, dt)
    }

    /// observe_at is `observe` for a sample that arrived late: the error read at a launch is the
    /// outcome of the placement the PREVIOUS launch's trim set, so `age` [s] counts in this channel's
    /// own `dt` (one sample per step) — `Adapt::learn_at`'s rule exactly.
    pub fn observe_at(&mut self, err_along: f64, dt: f64, age: f64) -> f64 {
        self.ada.learn_at("lead", err_along, age, dt)
    }

    /// scale is the multiplier a caller applies to the reference's lead terms: 1.0 with the trim at
    /// zero — the shipped behaviour, bit for bit, which is what makes a caller's A/B against the
    /// frozen arm a comparison of the LEARNING rather than of the wiring.
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
