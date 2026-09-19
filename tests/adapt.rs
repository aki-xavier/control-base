// adapt.rs — a layer's own arithmetic: where its integral rule comes to rest, what its limits do, and
// what it does with an error that has no bias in it. Engine-free.
// Assertions that need an absolute value spell it out: the rule approaches the null from BELOW, so a
// signed comparison would be satisfied by a trim that never arrives.

/// converge runs the layer against a synthetic plant whose error is a known linear function
/// of the trim, err(g) = e0 - c * g, so the fixed point e0 / c can be asserted directly.
use control_base::adapt::{Adapt, LeadTrim};

fn converge(rate: f64, e0: f64, c: f64, dt: f64, steps: usize, lo: f64, hi: f64) -> (f64, f64) {
    let mut a = Adapt::new();
    a.add("g", lo, hi, rate);
    let mut g = 0.0;
    let mut err = 0.0;
    for _ in 0..steps {
        err = e0 - c * g;
        g = a.learn("g", err, dt);
    }
    (g, err)
}

#[test]
fn the_integral_rule_comes_to_rest_at_the_null() {
    // the rule must reach the trim that cancels the error without being told what it is
    let e0 = 0.4;
    let c = 2.0;
    let want = e0 / c;
    let (got, err) = converge(2.0, e0, c, 1e-3, 40000, -10.0, 10.0);
    // the rule approaches the null from BELOW, so only the absolute value is not vacuously
    // satisfied (measured: it clears 5e-3 by twelve orders of magnitude)
    assert!(
        (got - want).abs() < 5e-3,
        "the trim settled at {got}, want {want}"
    );
    assert!(err > -5e-3 && err < 5e-3, "the error left is {err}");
    // the trim the layer holds starts at 0.0 (`add` writes it, `learn` returns it), so the 1000
    // steps below walk 0 up to the null: measured, g ends 3.6e-3 short, which is what the 1e-2
    // tolerance is sized for (0.996^1000 leaves 1.8% of the walk unrun)
    let mut a = Adapt::new();
    a.add("g", -10.0, 10.0, 2.0);
    let mut g = want;
    for _ in 0..1000 {
        g = a.learn("g", e0 - c * g, 1e-3);
    }
    assert!(
        (g - want).abs() < 1e-2,
        "the trim drifted to {g}, want {want}"
    );
}

#[test]
fn the_limits_stop_a_calibration_walking_off() {
    // a correction larger than the caller ever tested must not be reachable: a persistent error
    // drives the trim to its limit and stays there, and the error it still sees reports the ceiling
    let (got, err) = converge(20.0, 1.0, 0.001, 1e-3, 20000, -0.05, 0.05);
    assert_eq!(got, 0.05);
    assert!(err > 0.5, "the error at the ceiling is only {err}");
}

#[test]
fn a_zero_mean_error_does_not_accumulate() {
    // noise is not a bias: a zero-mean error must leave the trim where it was
    let mut a = Adapt::new();
    a.add("g", -1.0, 1.0, 5.0);
    let mut g = 0.0;
    for i in 0..20000 {
        let e = if i % 2 == 0 { 0.05 } else { -0.05 };
        g = a.learn("g", e, 1e-3);
    }
    assert!(g > -5e-3 && g < 5e-3, "the trim drifted to {g}");
    let rep = a.report();
    assert_eq!(rep.len(), 1);
    assert!(rep[0].contains("n=20000"), "the readout line is {}", rep[0]);
}

#[test]
fn an_unknown_parameter_is_inert() {
    // a caller that forgot to register a knob gets no correction instead of a
    // panic or an accidental learning rate
    let mut a = Adapt::new();
    a.add("known", -1.0, 1.0, 1.0);
    assert_eq!(a.trim("missing"), 0.0);
    assert_eq!(a.learn("missing", 1.0, 1e-3), 0.0);
    assert_eq!(a.steps, 0);
    assert_eq!(a.index("known"), Some(0));
    // an unknown parameter is None, not a -1 sentinel
    assert_eq!(a.index("missing"), None);
}

#[test]
fn a_frozen_parameter_still_reports() {
    // rate 0 keeps a knob recorded but untuned: the error is still measured and
    // reported, the trim never moves
    let mut a = Adapt::new();
    a.add("frozen", -1.0, 1.0, 0.0);
    for _ in 0..100 {
        a.learn("frozen", 0.3, 1e-3);
    }
    assert_eq!(a.trim("frozen"), 0.0);
    let i = a.index("frozen").expect("registered just above");
    assert_eq!(a.params[i].samples, 100);
    // the running mean is exactly 0.3 from the first sample, so the absolute value holds
    assert!((a.params[i].mean_err - 0.3).abs() < 1e-12);
    // the readout is pinned whole — the numbers a caller reads off it stay visible in the suite —
    // and byte for byte whatever the shortest round-tripping rendering of these values gives (rate 0.0
    // is where a `%.3g`-style rendering and the shortest form agree)
    assert_eq!(
        a.report()[0],
        "frozen: trim=0.0000 in [-1.000 1.000] rate=0 err=0.3000 mean=0.3000 n=100"
    );
}

/// A simultaneous credit IS the rule the immediate path already had, up to one rounding: `learn_at`
/// computes the credit at the parameter's OLD value and carries the difference forward, so with an age
/// inside one sample the two values agree except for the delta's add-then-subtract round-trip.
#[test]
fn a_simultaneous_delayed_credit_is_the_immediate_rule() {
    let mut a = Adapt::new();
    let mut b = Adapt::new();
    a.add("g", -10.0, 10.0, 2.0);
    b.add("g", -10.0, 10.0, 2.0);
    let mut worst = 0.0f64;
    for k in 0..2000 {
        let err = 0.3 * ((k as f64) * 0.017).sin();
        let ta = a.learn("g", err, 0.01);
        let tb = b.learn_at("g", err, 0.0, 0.01);
        worst = worst.max((ta - tb).abs());
    }
    assert!(
        worst < 1e-12,
        "the age-zero path must be the immediate rule, and it differs by {worst}"
    );
}

/// THE CREDIT IS EARNED AT THE VALUE THAT CAUSED IT, the case that tells the two rules apart: the
/// parameter is driven to its cap, where a credit is worth NOTHING, so a positive error credited one
/// sample late must move nothing — while credited to the value now it would earn a full step.
#[test]
fn a_delayed_credit_is_earned_at_the_value_that_caused_it() {
    let mut a = Adapt::new();
    a.add("g", -10.0, 0.5, 100.0);
    for _ in 0..50 {
        a.learn_at("g", 1.0, 0.0, 0.01);
    }
    assert!(
        (a.trim("g") - 0.5).abs() < 1e-12,
        "the trace's case needs the cap reached"
    );
    // one negative error, so the value NOW is below the cap while the value ONE SAMPLE AGO was at it
    let pulled = a.learn_at("g", -1.0, 0.0, 0.01);
    assert!(
        pulled < 0.5,
        "the negative error must pull the trim below its cap"
    );
    // and now the positive error, credited one sample late: the past value was the capped one
    let after = a.learn_at("g", 1.0, 0.01, 0.01);
    assert!(
        (after - pulled).abs() < 1e-12,
        "the credit belongs to the capped value, where it is worth nothing: the trim moved from \
         {pulled} to {after}, and a credit with no time in it would have raised it"
    );
    // the control: the same error credited to NOW does move the trim, which is what makes the
    // assertion above a measurement rather than an accident
    let mut b = Adapt::new();
    b.add("g", -10.0, 0.5, 100.0);
    for _ in 0..50 {
        b.learn_at("g", 1.0, 0.0, 0.01);
    }
    let pulled_b = b.learn_at("g", -1.0, 0.0, 0.01);
    let now = b.learn_at("g", 1.0, 0.0, 0.01);
    assert!(
        now > pulled_b + 1e-6,
        "the control side must move, or the discrimination above is vacuous ({pulled_b} -> {now})"
    );
}

/// THE TRACE IS BOUNDED: a constant, not a log, and a delay past its depth degrades to the CURRENT value.
#[test]
fn the_trace_is_bounded_and_its_depth_degrades_to_now() {
    let mut a = Adapt::new();
    a.add("g", -10.0, 10.0, 1.0);
    for _ in 0..(control_base::adapt::TRACE + 20) {
        a.learn("g", 0.1, 0.01);
    }
    assert_eq!(
        a.params[0].trace.len(),
        control_base::adapt::TRACE,
        "the trace must stop growing at its own depth"
    );
    // a delay far past the depth: the credit falls back to the current value, which is the immediate
    // rule, so the failure mode of a wrong age is a lost correction and never a wrong number
    let before = a.trim("g");
    let after = a.learn_at("g", 0.0, 1e6, 0.01);
    assert!(
        (after - before).abs() < 1e-12,
        "a zero error must move nothing"
    );
}

/// THE LEARNED FEEDFORWARD'S OWN ARITHMETIC.
///
/// `LeadTrim` is the smallest thing that is a learned inverse model: a trim on a reference's own
/// lead (its velocity and acceleration terms), driven by the movement's along-track error. What this
/// pins is the arithmetic, for the machine that later has a gap for it to close: the scale is exactly
/// 1 at a zero trim (the behaviour with no learning in it, bit for bit), the trim integrates the error
/// the caller hands in, it is bounded on both sides, and a frozen one stays where it is.
#[test]
fn the_lead_trim_integrates_the_along_track_error_and_is_bounded() {
    // (1) A ZERO TRIM LEAVES THE LEAD UNCHANGED: scale 1.0 exactly, whatever the rate
    for rate in [0.0f64, 2.0] {
        let lt = LeadTrim::new(rate, 1.0);
        assert_eq!(lt.scale(), 1.0);
        assert_eq!(lt.ada.trim("lead"), 0.0);
    }
    // (2) A PERSISTENT "BEHIND ITS REFERENCE" ERROR RAISES THE SCALE, which is the orientation
    // contract the caller owes this layer
    let mut lt = LeadTrim::new(2.0, 1.0);
    let mut last = 1.0;
    for _ in 0..2000 {
        lt.observe(0.002, 1e-3);
        let s = lt.scale();
        assert!(s > last, "the scale did not follow a persistent lag");
        last = s;
    }
    assert!(
        (lt.ada.trim("lead") - 2.0 * 0.002 * 2000.0 * 1e-3).abs() < 1e-12,
        "the trim is {}, not rate * err * t",
        lt.ada.trim("lead")
    );
    // (3) AND IT IS BOUNDED ON BOTH SIDES, by the cap the constructor was given
    let mut lt = LeadTrim::new(2000.0, 1.0);
    for _ in 0..1000 {
        lt.observe(0.01, 1e-3);
    }
    assert_eq!(lt.ada.trim("lead"), 1.0);
    assert_eq!(lt.scale(), 2.0);
    let mut lt2 = LeadTrim::new(2000.0, 1.0);
    for _ in 0..1000 {
        lt2.observe(-0.01, 1e-3);
    }
    assert_eq!(lt2.ada.trim("lead"), -1.0);
    assert_eq!(lt2.scale(), 0.0);
    // (4) A FROZEN ONE STAYS WHERE IT IS, which is the control an A/B needs
    let mut frozen = LeadTrim::new(0.0, 1.0);
    for _ in 0..1000 {
        frozen.observe(0.05, 1e-3);
    }
    assert_eq!(frozen.scale(), 1.0);
}

/// THE SAMPLING DISCIPLINE IS AUDITED: a channel is fed once per ITS OWN event with that event's
/// interval, and both mistakes a per-step channel invites show up in the span stream — `dt = 0` counts
/// samples and learns nothing, the tick period where the error arrives once per step under-learns.
/// The audit refuses nothing (the layer stays arithmetic); it makes the mistake readable.
#[test]
fn the_sampling_discipline_is_audited() {
    let mut a = Adapt::new();
    a.add("per_step", -1.0, 1.0, 1.0);
    for k in 0..50 {
        a.learn("per_step", 0.1, 0.35);
        assert_eq!(a.params[0].suspect, 0, "a clean channel was flagged at {k}");
    }
    assert!((a.params[0].mean_span - 0.35).abs() < 1e-12);
    // (2) the per-tick mistake: after the per-step baseline is established, a fast sample is counted
    a.learn("per_step", 0.1, 2e-4);
    assert_eq!(a.params[0].suspect, 1, "the fast sample was not flagged");
    // (3) the `dt = 0` mistake, a channel that counted samples and learned nothing: flagged on its
    // face, warmup or not
    let mut z = Adapt::new();
    z.add("zero", -1.0, 1.0, 1.0);
    z.learn("zero", 0.5, 0.0);
    assert_eq!(z.params[0].suspect, 1);
    assert_eq!(z.params[0].trim, 0.0, "a zero span learned something");
    // (4) a non-finite span neither learns nor poisons the mean
    z.learn("zero", 0.5, f64::NAN);
    assert_eq!(z.params[0].suspect, 2);
    assert!(z.params[0].mean_span.is_finite());
    // (5) and the audit is readable where the channels are: one line per channel, with the numbers
    let lines = a.span_report();
    assert_eq!(lines.len(), 1);
    assert!(lines[0].contains("suspect=1"), "{}", lines[0]);
    assert!(lines[0].contains("mean_span=0.35") || lines[0].contains("mean_span=0.3"));
    // while `report`'s own text is untouched (it is pinned byte for byte elsewhere in this file)
    assert!(!a.report()[0].contains("suspect"));
}

/// THE CONVERGENCE THE LAYER'S OWN TIMING STRUCTURE EXISTS FOR: an error delivered N samples late
/// must still converge, for a STATED range of N, to the same value it reaches with no delay.
///
/// The plant is the file's opening `converge` one with the delay where the machine's own chain has
/// it: the error at sample k is `e0 - c * g`, `g` being the trim the plant was DRIVEN with N samples
/// earlier. N = 1..=8 brackets the one-step chains a per-step channel is reached through. No output:
/// the numbers are the assertions.
#[test]
fn the_aged_credit_converges_over_a_stated_range_of_delays() {
    let (e0, c) = (0.4, 2.0);
    let want = e0 / c;
    let dt = 0.35; // a per-step channel's own sample: one SAMPLE per step, 0.35 s each
    const STEPS: usize = 400;
    for n in 1..=8usize {
        // the aged channel: each error is the outcome of the trim the plant drove with n samples ago,
        // and `learn_at` credits it to exactly that value
        let mut aged = Adapt::new();
        aged.add("g", -10.0, 10.0, 0.1);
        let mut g: Vec<f64> = vec![0.0];
        for _ in 0..STEPS {
            let k = g.len();
            let drove = if k >= n { g[k - n] } else { 0.0 };
            let err = e0 - c * drove;
            g.push(aged.learn_at("g", err, n as f64 * dt, dt));
        }
        // the immediate rule on the SAME delayed plant: the same errors, credited to the value now
        let mut now = Adapt::new();
        now.add("g", -10.0, 10.0, 0.1);
        let mut h: Vec<f64> = vec![0.0];
        for _ in 0..STEPS {
            let k = h.len();
            let drove = if k >= n { h[k - n] } else { 0.0 };
            let err = e0 - c * drove;
            h.push(now.learn("g", err, dt));
        }
        let (a, b) = (aged.trim("g"), now.trim("g"));
        assert!(
            (a - want).abs() < 1e-9,
            "N = {n}: the aged credit settled at {a}, want {want}"
        );
        assert!(
            (a - b).abs() < 1e-9,
            "N = {n}: the aged credit ({a}) and the immediate one under the same delay ({b}) must \
             converge to the same answer"
        );
    }
    // AND WITH NO DELAY AT ALL the rule lands in the same place: the delay moves the road, not the
    // destination
    let mut plain = Adapt::new();
    plain.add("g", -10.0, 10.0, 0.1);
    let mut p: Vec<f64> = vec![0.0];
    for _ in 0..STEPS {
        let err = e0 - c * p[p.len() - 1];
        p.push(plain.learn("g", err, dt));
    }
    assert!(
        (plain.trim("g") - want).abs() < 1e-9,
        "the undelayed rule settled at {}, want {want}",
        plain.trim("g")
    );
}

/// AND `LeadTrim::observe_at` HAS THE SAME PROPERTY: it is the same rule reached through a per-step
/// channel (the error read at one step is the outcome of the placement the PREVIOUS step's trim set).
/// Two claims: the credit LANDS ON THE VALUE THAT PRODUCED THE ERROR — a trim held AT its cap has no
/// room for a further credit, so an aged credit that moves nothing there was earned at the capped
/// value — and the trim CONVERGES to the immediate rule's answer under the same delay, over the same
/// N = 1..=8.
#[test]
fn an_aged_lead_credit_lands_on_its_cause_and_converges() {
    // (1) THE CREDIT LANDS ON THE VALUE THAT CAUSED IT
    for n in 1..=8usize {
        let mut lt = LeadTrim::new(100.0, 1.0);
        for _ in 0..4 {
            lt.observe_at(1.0, 0.01, 0.0);
        }
        assert_eq!(lt.ada.trim("lead"), 1.0, "the cap must be reached first");
        // one negative error, so the value NOW is below the cap while the value one sample ago was
        // at it, then n-1 no-op samples so the value n samples back is still the capped one
        let pulled = lt.observe_at(-2.0, 0.01, 0.0);
        assert!(
            pulled < 1.0,
            "the negative error must pull the trim below its cap"
        );
        for _ in 0..(n - 1) {
            lt.observe_at(0.0, 0.01, 0.0);
        }
        let after = lt.observe_at(1.0, 0.01, n as f64 * 0.01);
        assert!(
            (after - pulled).abs() < 1e-12,
            "N = {n}: the credit belongs to the capped value, where it is worth nothing \
             ({pulled} -> {after}); a credit with no time in it would have raised it"
        );
    }
    // (2) AND IT CONVERGES, to the same answer the immediate rule reaches under the same delay
    let (e0, c) = (0.4, 2.0);
    let want = e0 / c;
    let dt = 0.35; // a per-step channel's own sample: one sample per step
    const STEPS: usize = 400;
    for n in 1..=8usize {
        let mut aged = LeadTrim::new(0.1, 1.0);
        let mut g: Vec<f64> = vec![0.0];
        for _ in 0..STEPS {
            let k = g.len();
            let drove = if k >= n { g[k - n] } else { 0.0 };
            let err = e0 - c * drove;
            g.push(aged.observe_at(err, dt, n as f64 * dt));
        }
        let mut now = LeadTrim::new(0.1, 1.0);
        let mut h: Vec<f64> = vec![0.0];
        for _ in 0..STEPS {
            let k = h.len();
            let drove = if k >= n { h[k - n] } else { 0.0 };
            let err = e0 - c * drove;
            h.push(now.observe(err, dt));
        }
        let (a, b) = (aged.ada.trim("lead"), now.ada.trim("lead"));
        assert!(
            (a - want).abs() < 1e-9,
            "N = {n}: the aged lead settled at {a}, want {want}"
        );
        assert!(
            (a - b).abs() < 1e-9,
            "N = {n}: the aged lead ({a}) and the immediate one under the same delay ({b}) must \
             converge to the same answer"
        );
    }
}
