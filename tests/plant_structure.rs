// plant_structure.rs — the Plant contract's EXPRESSIVE RANGE, as a check rather than a promise.
//
// The contract this replaces stated one machine's shape and left it unsaid: no base in the state
// silently meant the base was welded, `compute_jacobian` answered for a frame nothing named, and a
// machine whose base floats could not implement the trait at all. `PlantStructure` is what makes the
// difference STATABLE, and these are the two statements that matter — a welded chain is braced by
// construction, and a floating machine's brace has to come from its contacts, where a friction
// contact is not a weld and does not substitute for one.

use control_base::plant::{ContactKind, ExternalContact, PlantStructure};

/// bodies is a small named chain, so the tests read like a machine rather than like indices.
fn bodies() -> Vec<String> {
    ["l1", "l2", "l3"].iter().map(|s| s.to_string()).collect()
}

#[test]
fn a_welded_chain_declares_itself_braced_by_construction() {
    let s = PlantStructure::fixed_base(3, bodies(), "tip".to_string());
    assert_eq!(s.dof, 3, "the arm's DOF is its joints");
    assert_eq!(s.base_dof, 0, "a welded base is not a state");
    assert!(s.all_driven(), "every joint of a serial arm is driven");
    assert!(!s.base_is_a_state());
    assert!(
        s.braced(),
        "the world holds the welded base in all six directions"
    );
    assert!(
        s.contacts.is_empty(),
        "a welded base needs no distal contact"
    );
    assert!(s.has_body("l2") && !s.has_body("l9"));
    assert_eq!(
        s.task_frame, "tip",
        "the task frame is a NAME, not an assumption"
    );
}

#[test]
fn a_floating_base_is_a_state_whose_brace_must_be_supplied() {
    // 6 base coordinates + 3 joints: the base floats and the joints are the driven ones, which is
    // the shape a humanoid is and the shape the old contract could not state at all.
    let mut s = PlantStructure {
        dof: 9,
        actuated: vec![false, false, false, false, false, false, true, true, true],
        base_dof: 6,
        contacts: Vec::new(),
        bodies: bodies(),
        task_frame: "tip".to_string(),
    };
    assert!(s.base_is_a_state());
    assert!(!s.all_driven(), "the base's six coordinates are undriven");
    assert!(
        !s.braced(),
        "a floating machine with no contact has nothing to write a wrench demand against"
    );

    // A foot does NOT substitute for a weld: it holds one direction, and only in compression.
    s.contacts = vec![ExternalContact {
        frame: "foot_l".to_string(),
        kind: ContactKind::Friction { mu: 0.6 },
    }];
    assert!(
        !s.braced(),
        "a friction contact is not a weld — the regime it names has an infeasible side, and on that \
         side the contact breaks rather than saturates"
    );

    // Only a distal WELD braces a floating machine, which is what a hand fixed to a rail would be.
    s.contacts.push(ExternalContact {
        frame: "hand".to_string(),
        kind: ContactKind::Weld,
    });
    assert!(s.braced(), "a welded distal contact is a brace");
}
