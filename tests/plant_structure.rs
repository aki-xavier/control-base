use control_base::plant::{ContactKind, ExternalContact, Footprint, PlantStructure, TaskMap};

fn bodies() -> Vec<String> {
    ["l1", "l2", "l3"].iter().map(|s| s.to_string()).collect()
}

fn tip() -> TaskMap {
    TaskMap::Frame("tip".to_string())
}

#[test]
fn a_welded_chain_declares_itself_braced_by_construction() {
    let s = PlantStructure::fixed_base(3, bodies(), tip());
    assert_eq!(s.dof, 3, "the DOF is its joints");
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
        s.task_map,
        tip(),
        "the task is a KIND and a name, not an assumption"
    );
    assert!(!s.task_is_a_point());
    assert!(
        !s.has_point("com"),
        "a welded chain's points of interest are all on links, where a frame answers for them"
    );
}

#[test]
fn a_floating_base_is_a_state_whose_brace_must_be_supplied() {
    let mut s = PlantStructure {
        dof: 9,
        actuated: vec![false, false, false, false, false, false, true, true, true],
        base_dof: 6,
        contacts: Vec::new(),
        bodies: bodies(),
        points: Vec::new(),
        task_map: tip(),
    };
    assert!(s.base_is_a_state());
    assert!(!s.all_driven(), "the base's six coordinates are undriven");
    assert!(
        !s.braced(),
        "a floating machine with no contact has nothing to write a wrench demand against"
    );

    s.contacts = vec![ExternalContact {
        frame: "foot_l".to_string(),
        kind: ContactKind::Friction {
            mu: 0.6,
            footprint: Footprint::POINT,
        },
    }];
    assert!(
        !s.braced(),
        "a friction contact is not a weld — the regime it names has an infeasible side, and on that \
         side the contact breaks rather than saturates"
    );

    s.contacts.push(ExternalContact {
        frame: "hand".to_string(),
        kind: ContactKind::Weld,
    });
    assert!(s.braced(), "a welded distal contact is a brace");
}

#[test]
fn a_task_may_be_a_point_and_then_it_has_no_rotation_to_report() {
    let mut s = PlantStructure {
        dof: 9,
        actuated: vec![false, false, false, false, false, false, true, true, true],
        base_dof: 6,
        contacts: Vec::new(),
        bodies: bodies(),
        points: vec!["com".to_string()],
        task_map: TaskMap::Point("com".to_string()),
    };
    assert!(s.task_is_a_point(), "the task's KIND is data, not a guess");
    assert_eq!(s.task_map.name(), "com");
    assert!(s.has_point("com") && !s.has_point("l1"));
    assert!(
        !s.has_body("com"),
        "a point is not a body: it is a function of the whole configuration, not a link's distal end"
    );
    s.task_map = tip();
    assert!(!s.task_is_a_point());
}
