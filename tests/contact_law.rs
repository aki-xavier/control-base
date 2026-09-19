// contact_law.rs — the laws' own arithmetic, as checks: what each law commands and where it is silent,
// what it does with a report and a geometry that disagree, and what the complementarity solve does to a
// system whose answer is known by hand. The plants' suites pin the NUMBERS a machine produces under a
// law; this one pins the laws.

use control_base::contact_injection::ContactInjection;
use control_base::contact_law::{
    ContactConstraint, ContactLaw, ContactSlot, ContactSystem, FlooredContact, PenaltyContact,
    ReportContact, SolveContact,
};
use control_math::mat::Mat;

/// slot is one slot with the numbers a plant hands over: the injection (this slot's own, already shared
/// out over the set), the world's report and gate, the penetration of each point, this side's stiffness
/// and whether the set is this side's own.
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

/// The report law is the world's word: the readout for the slot, shared over the set, clamped to what
/// the slot may command. It is the arm's contact, and the clamp is where a readout becomes a force.
#[test]
fn the_report_law_takes_the_world_at_its_word() {
    let inj = ContactInjection::new(0.0, 300.0, 150.0, 0.0);
    let law = ReportContact;
    let mut out = Vec::new();

    // one point, no geometry of this side's (the arm's slot): the report IS the force
    assert!(law.decide_into(
        &slot(&inj, [10.0, -20.0, 150.0], true, &[0.0], 0.0, false),
        &mut out
    ));
    assert_eq!(out, vec![[10.0, -20.0, 150.0]]);

    // the clamp is per component, in both directions
    assert!(law.decide_into(
        &slot(&inj, [1000.0, -1000.0, 50.0], true, &[0.0], 0.0, false),
        &mut out
    ));
    assert_eq!(out, vec![[300.0, -300.0, 50.0]]);

    // a set of four points shares the report evenly: what the slot carries is the readout either way
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

    // a slot the world does not report is left alone, and the caller's buffer is cleared rather than
    // holding the last slot's forces
    out.push([1.0, 1.0, 1.0]);
    assert!(!law.decide_into(
        &slot(&inj, [0.0, 0.0, 0.0], false, &[0.0], 0.0, false),
        &mut out
    ));
    assert!(
        out.is_empty(),
        "a silent slot left {out:?} in the caller's buffer"
    );
}

/// The penalty law is this side's geometry alone: `k * pen / n` at each point, the report ignored
/// entirely, and a slot silent only when its own geometry says nothing.
#[test]
fn the_penalty_law_is_its_own_geometry_only() {
    let inj = ContactInjection::new(0.0, 600.0, 0.0, 0.05);
    let law = PenaltyContact;
    let mut out = Vec::new();

    // two points at 1 mm and 3 mm inside a 63 kN/m sole: each carries its own depth over the set's size,
    // so the slot's total is k * sum(pen) / n
    assert!(law.decide_into(
        &slot(&inj, [0.0, 0.0, 0.0], false, &[0.001, 0.003], 63000.0, true),
        &mut out
    ));
    assert_eq!(out, vec![[0.0, 0.0, 31.5], [0.0, 0.0, 94.5]]);
    assert_eq!(out[0][2] + out[1][2], 126.0);

    // a report is NOT this law's input: a slot carrying 500 N of world force above the plane commands
    // nothing (the geometry is what it reads, and the geometry is not in contact)
    assert!(!law.decide_into(
        &slot(&inj, [500.0, 0.0, 500.0], true, &[0.0], 63000.0, true),
        &mut out
    ));

    // a machine with no stiffness of its own (a slot whose geometry is the world's) commands nothing
    assert!(!law.decide_into(
        &slot(&inj, [0.0, 0.0, 500.0], true, &[0.01], 0.0, false),
        &mut out
    ));
}

/// The raised law is the report floored by the geometry — the larger of the two, point by point — and
/// the case it exists for is the one where the two disagree: the world has stopped reporting while the
/// foot is visibly in the floor.
#[test]
fn the_floored_law_raises_the_report_to_the_geometry() {
    let inj = ContactInjection::new(0.005, 600.0, 210.0, 0.05);
    let law = FlooredContact;
    let mut out = Vec::new();

    // two points at 2 mm, a 63 kN/m sole, and a report of 100 N over the slot: each point's own penalty
    // (63.0 N) is above its share of the report (50.0 N), so the geometry is what is injected
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

    // and the report wins where it is the larger one: 400 N over two points is 200 N each
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

    // THE CASE IT EXISTS FOR: the report has gone quiet and the soles are 2 mm inside the plate, so the
    // slot is live on its geometry alone (which the world's own gate cannot see)
    assert!(law.decide_into(
        &slot(&inj, [0.0, 0.0, 0.0], false, &[0.002, 0.002], 63000.0, true),
        &mut out
    ));
    assert_eq!(out, vec![[0.0, 0.0, 63.0], [0.0, 0.0, 63.0]]);

    // both sources silent: nothing is commanded
    assert!(!law.decide_into(
        &slot(&inj, [0.0, 0.0, 0.0], false, &[0.0, 0.0], 63000.0, true),
        &mut out
    ));

    // the clamp is NOT applied to this law's forces — the documented difference from the report law. A
    // readout of 4000 N over two points is 2000 N each, well past the slot's 600 N bound.
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

/// The solve is a statement about the machine's own system, so its answer is checked against one whose
/// answer is arithmetic: a unit mass moving down at 0.5 m/s, stopped AT the plane by the impulse the
/// constraint requires, and a patch that must pick which of its points carries the load.
#[test]
fn the_solve_stops_a_landing_at_the_plane() {
    let law = SolveContact::default();
    assert_eq!(law.beta, 0.2);
    assert_eq!(law.iters, 20);

    let inj = ContactInjection::new(0.0, 600.0, 0.0, 0.05);
    // this scheme takes the sets this side declares, and only those
    assert!(law.may_decide(&slot(&inj, [0.0; 3], true, &[0.0], 63000.0, true)));
    assert!(!law.may_decide(&slot(&inj, [0.0; 3], true, &[0.0], 63000.0, false)));

    // ONE DEGREE OF FREEDOM, landing: M = 1, no forces, velocity -0.5 m/s, dt = 10 ms. The multiplier
    // that makes the normal velocity zero is 0.5 N.s, i.e. 50 N over the step.
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

    // THE SAME FOOT, ALREADY 2 mm INSIDE: the bound is the relaxed one (`beta * pen / dt`), so the
    // solve pushes out a fifth of the penetration per step rather than flinging it at 0.2 m/s
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

    // A FOOT IN THE AIR ASKS FOR NOTHING: a point above the plane has a NEGATIVE bound, so its
    // multiplier comes out zero on its own
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

/// One patch, one rigid body, two points: the coupling that makes a solved contact a solve rather than
/// two independent springs — and the deterministic tie-break the projection has, which is what makes a
/// machine's pressure point reproducible.
#[test]
fn a_solved_patch_shares_its_load_through_the_machine() {
    let law = SolveContact::default();
    // two DOFs, both fed by the SAME normal direction: an impulse at either point decelerates the body
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
    // the first point swept takes the whole 0.4 N.s the 0.2 m/s stop needs (each point buys 0.5 m/s per
    // N.s, and the second then asks for a bound it has already met); 0.4 N.s over 10 ms is 40 N against
    // the 2 kg * 0.2 m/s the body carried
    assert_eq!(lam, vec![0.4, 0.0]);
    assert!((lam[0] - 2.0 * 0.2 / 0.01 * 0.01).abs() < 1e-12);

    // and the solve of an empty candidate list is the empty answer, not a panic
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
