use control_base::contact_injection::ContactInjection;
use control_base::contact_law::{
    ContactConstraint, ContactLaw, ContactSlot, ContactSystem, FlooredContact, PenaltyContact,
    ReportContact, SolveContact,
};
use control_math::mat::Mat;

fn slot<'a>(
    inj: &'a ContactInjection,
    report: [f64; 3],
    live: bool,
    pen: &'a [f64],
    k: f64,
    own_set: bool,
) -> ContactSlot<'a> {
    ContactSlot {
        inj,
        report,
        live,
        pen,
        k,
        own_set,
    }
}

#[test]
fn the_report_law_takes_the_world_at_its_word() {
    let inj = ContactInjection::new(0.0, 300.0, 150.0, 0.0);
    let law = ReportContact;
    let mut out = Vec::new();

    assert!(law.decide_into(
        &slot(&inj, [10.0, -20.0, 150.0], true, &[0.0], 0.0, false),
        &mut out
    ));
    assert_eq!(out, vec![[10.0, -20.0, 150.0]]);

    assert!(law.decide_into(
        &slot(&inj, [1000.0, -1000.0, 50.0], true, &[0.0], 0.0, false),
        &mut out
    ));
    assert_eq!(out, vec![[300.0, -300.0, 50.0]]);

    assert!(law.decide_into(
        &slot(
            &inj,
            [0.0, 0.0, 80.0],
            true,
            &[0.0, 0.0, 0.0, 0.0],
            0.0,
            false
        ),
        &mut out
    ));
    assert_eq!(out.len(), 4);
    for f in &out {
        assert_eq!(*f, [0.0, 0.0, 20.0]);
    }

    out.push([1.0, 1.0, 1.0]);
    assert!(!law.decide_into(
        &slot(&inj, [0.0, 0.0, 0.0], false, &[0.0], 0.0, false),
        &mut out
    ));
    assert!(out.is_empty(), "a silent slot left {out:?} in the buffer");
}

#[test]
fn the_penalty_law_is_its_own_geometry_only() {
    let inj = ContactInjection::new(0.0, 600.0, 0.0, 0.05);
    let law = PenaltyContact;
    let mut out = Vec::new();

    assert!(law.decide_into(
        &slot(&inj, [0.0, 0.0, 0.0], false, &[0.001, 0.003], 63000.0, true),
        &mut out
    ));
    assert_eq!(out, vec![[0.0, 0.0, 31.5], [0.0, 0.0, 94.5]]);
    assert_eq!(out[0][2] + out[1][2], 126.0);

    assert!(!law.decide_into(
        &slot(&inj, [500.0, 0.0, 500.0], true, &[0.0], 63000.0, true),
        &mut out
    ));

    assert!(!law.decide_into(
        &slot(&inj, [0.0, 0.0, 500.0], true, &[0.01], 0.0, false),
        &mut out
    ));
}

#[test]
fn the_floored_law_raises_the_report_to_the_geometry() {
    let inj = ContactInjection::new(0.005, 600.0, 210.0, 0.05);
    let law = FlooredContact;
    let mut out = Vec::new();

    assert!(law.decide_into(
        &slot(
            &inj,
            [0.0, 0.0, 100.0],
            true,
            &[0.002, 0.002],
            63000.0,
            true
        ),
        &mut out
    ));
    assert_eq!(out, vec![[0.0, 0.0, 63.0], [0.0, 0.0, 63.0]]);

    assert!(law.decide_into(
        &slot(
            &inj,
            [0.0, 0.0, 400.0],
            true,
            &[0.002, 0.002],
            63000.0,
            true
        ),
        &mut out
    ));
    assert_eq!(out, vec![[0.0, 0.0, 200.0], [0.0, 0.0, 200.0]]);

    assert!(law.decide_into(
        &slot(&inj, [0.0, 0.0, 0.0], false, &[0.002, 0.002], 63000.0, true),
        &mut out
    ));
    assert_eq!(out, vec![[0.0, 0.0, 63.0], [0.0, 0.0, 63.0]]);

    assert!(!law.decide_into(
        &slot(&inj, [0.0, 0.0, 0.0], false, &[0.0, 0.0], 63000.0, true),
        &mut out
    ));

    assert!(law.decide_into(
        &slot(
            &inj,
            [0.0, 0.0, 4000.0],
            true,
            &[0.002, 0.002],
            63000.0,
            true
        ),
        &mut out
    ));
    assert_eq!(out, vec![[0.0, 0.0, 2000.0], [0.0, 0.0, 2000.0]]);
}

#[test]
fn the_solve_stops_a_landing_at_the_plane() {
    let law = SolveContact::default();
    assert_eq!(law.beta, 0.2);
    assert_eq!(law.iters, 20);

    let inj = ContactInjection::new(0.0, 600.0, 0.0, 0.05);
    assert!(law.may_decide(&slot(&inj, [0.0; 3], true, &[0.0], 63000.0, true)));
    assert!(!law.may_decide(&slot(&inj, [0.0; 3], true, &[0.0], 63000.0, false)));

    let mass = Mat::from_rows(&[vec![1.0]]);
    let cands = vec![ContactConstraint {
        jn: vec![1.0],
        pen: 0.0,
    }];
    let sys = ContactSystem {
        mass: &mass,
        rhs: &[0.0],
        vel: &[-0.5],
        dt: 0.01,
        candidates: &cands,
    };
    let mut lam = Vec::new();
    law.solve_into(&sys, &mut lam);
    assert_eq!(lam.len(), 1);
    assert!(
        (lam[0] - 0.5).abs() < 1e-12,
        "the landing impulse is {} N.s, not the 50 N the step needs",
        lam[0]
    );

    let cands = vec![ContactConstraint {
        jn: vec![1.0],
        pen: 0.002,
    }];
    let sys = ContactSystem {
        mass: &mass,
        rhs: &[0.0],
        vel: &[0.0],
        dt: 0.01,
        candidates: &cands,
    };
    law.solve_into(&sys, &mut lam);
    assert!(
        (lam[0] - 0.04).abs() < 1e-12,
        "a 2 mm penetration under beta = 0.2 is {} N.s, not the 4 N the relaxation asks for",
        lam[0]
    );

    let cands = vec![ContactConstraint {
        jn: vec![1.0],
        pen: -0.05,
    }];
    let sys = ContactSystem {
        mass: &mass,
        rhs: &[0.0],
        vel: &[0.0],
        dt: 0.01,
        candidates: &cands,
    };
    law.solve_into(&sys, &mut lam);
    assert_eq!(lam[0], 0.0);
}

#[test]
fn a_solved_patch_shares_its_load_through_the_machine() {
    let law = SolveContact::default();
    let mass = Mat::from_rows(&[vec![2.0, 0.0], vec![0.0, 2.0]]);
    let cands = vec![
        ContactConstraint {
            jn: vec![1.0, 0.0],
            pen: 0.0,
        },
        ContactConstraint {
            jn: vec![1.0, 0.0],
            pen: 0.0,
        },
    ];
    let sys = ContactSystem {
        mass: &mass,
        rhs: &[0.0, 0.0],
        vel: &[-0.2, 0.0],
        dt: 0.01,
        candidates: &cands,
    };
    let mut lam = Vec::new();
    law.solve_into(&sys, &mut lam);
    assert_eq!(lam, vec![0.4, 0.0]);
    assert!((lam[0] - 2.0 * 0.2 / 0.01 * 0.01).abs() < 1e-12);

    let sys = ContactSystem {
        mass: &mass,
        rhs: &[0.0, 0.0],
        vel: &[0.0, 0.0],
        dt: 0.01,
        candidates: &[],
    };
    lam.push(1.0);
    law.solve_into(&sys, &mut lam);
    assert!(lam.is_empty());
}
