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
    let e0 = 0.4;
    let c = 2.0;
    let want = e0 / c;
    let (got, err) = converge(2.0, e0, c, 1e-3, 40000, -10.0, 10.0);
    assert!(
        (got - want).abs() < 5e-3,
        "the trim settled at {got}, want {want}"
    );
    assert!(err > -5e-3 && err < 5e-3, "the error left is {err}");
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
    let (got, err) = converge(20.0, 1.0, 0.001, 1e-3, 20000, -0.05, 0.05);
    assert_eq!(got, 0.05);
    assert!(err > 0.5, "the error at the ceiling is only {err}");
}

#[test]
fn a_zero_mean_error_does_not_accumulate() {
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
    let mut a = Adapt::new();
    a.add("known", -1.0, 1.0, 1.0);
    assert_eq!(a.trim("missing"), 0.0);
    assert_eq!(a.learn("missing", 1.0, 1e-3), 0.0);
    assert_eq!(a.steps, 0);
    assert_eq!(a.index("known"), Some(0));
    assert_eq!(a.index("missing"), None);
}

#[test]
fn a_frozen_parameter_still_reports() {
    let mut a = Adapt::new();
    a.add("frozen", -1.0, 1.0, 0.0);
    for _ in 0..100 {
        a.learn("frozen", 0.3, 1e-3);
    }
    assert_eq!(a.trim("frozen"), 0.0);
    let i = a.index("frozen").expect("registered just above");
    assert_eq!(a.params[i].samples, 100);
    assert!((a.params[i].mean_err - 0.3).abs() < 1e-12);
    assert_eq!(
        a.report()[0],
        "frozen: trim=0.0000 in [-1.000 1.000] rate=0 err=0.3000 mean=0.3000 n=100"
    );
}

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
    let pulled = a.learn_at("g", -1.0, 0.0, 0.01);
    assert!(
        pulled < 0.5,
        "the negative error must pull the trim below its cap"
    );
    let after = a.learn_at("g", 1.0, 0.01, 0.01);
    assert!(
        (after - pulled).abs() < 1e-12,
        "the credit belongs to the capped value, where it is worth nothing: the trim moved from \
         {pulled} to {after}, and a credit with no time in it would have raised it"
    );
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
    let before = a.trim("g");
    let after = a.learn_at("g", 0.0, 1e6, 0.01);
    assert!(
        (after - before).abs() < 1e-12,
        "a zero error must move nothing"
    );
}

#[test]
fn the_lead_trim_integrates_the_along_track_error_and_is_bounded() {
    for rate in [0.0f64, 2.0] {
        let lt = LeadTrim::new(rate, 1.0);
        assert_eq!(lt.scale(), 1.0);
        assert_eq!(lt.ada.trim("lead"), 0.0);
    }
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
    let mut frozen = LeadTrim::new(0.0, 1.0);
    for _ in 0..1000 {
        frozen.observe(0.05, 1e-3);
    }
    assert_eq!(frozen.scale(), 1.0);
}

#[test]
fn the_sampling_discipline_is_audited() {
    let mut a = Adapt::new();
    a.add("per_step", -1.0, 1.0, 1.0);
    for k in 0..50 {
        a.learn("per_step", 0.1, 0.35);
        assert_eq!(a.params[0].suspect, 0, "a clean channel was flagged at {k}");
    }
    assert!((a.params[0].mean_span - 0.35).abs() < 1e-12);
    a.learn("per_step", 0.1, 2e-4);
    assert_eq!(a.params[0].suspect, 1, "the fast sample was not flagged");
    let mut z = Adapt::new();
    z.add("zero", -1.0, 1.0, 1.0);
    z.learn("zero", 0.5, 0.0);
    assert_eq!(z.params[0].suspect, 1);
    assert_eq!(z.params[0].trim, 0.0, "a zero span learned something");
    z.learn("zero", 0.5, f64::NAN);
    assert_eq!(z.params[0].suspect, 2);
    assert!(z.params[0].mean_span.is_finite());
    let lines = a.span_report();
    assert_eq!(lines.len(), 1);
    assert!(lines[0].contains("suspect=1"), "{}", lines[0]);
    assert!(lines[0].contains("mean_span=0.35") || lines[0].contains("mean_span=0.3"));
    assert!(!a.report()[0].contains("suspect"));
}

#[test]
fn the_aged_credit_converges_over_a_stated_range_of_delays() {
    let (e0, c) = (0.4, 2.0);
    let want = e0 / c;
    let dt = 0.35;
    const STEPS: usize = 400;
    for n in 1..=8usize {
        let mut aged = Adapt::new();
        aged.add("g", -10.0, 10.0, 0.1);
        let mut g: Vec<f64> = vec![0.0];
        for _ in 0..STEPS {
            let k = g.len();
            let drove = if k >= n { g[k - n] } else { 0.0 };
            let err = e0 - c * drove;
            g.push(aged.learn_at("g", err, n as f64 * dt, dt));
        }
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

#[test]
fn an_aged_lead_credit_lands_on_its_cause_and_converges() {
    for n in 1..=8usize {
        let mut lt = LeadTrim::new(100.0, 1.0);
        for _ in 0..4 {
            lt.observe_at(1.0, 0.01, 0.0);
        }
        assert_eq!(lt.ada.trim("lead"), 1.0, "the cap must be reached first");
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
    let (e0, c) = (0.4, 2.0);
    let want = e0 / c;
    let dt = 0.35;
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
