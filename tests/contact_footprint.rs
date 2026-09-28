use control_base::plant::Footprint;
use control_math::vec3::Vec3;

fn sole() -> Footprint {
    Footprint {
        offset: Vec3::new(0.038, 0.0, -0.035),
        back: 0.058,
        front: 0.134,
        half_w: 0.034,
    }
}

#[test]
fn a_point_contact_holds_no_moment() {
    let p = Footprint::POINT;
    assert!(!p.clamp(0.0, 0.0).2);
    assert_eq!(p.clamp(0.01, -0.02), (0.0, 0.0, true));
    assert_eq!(p.margin(0.0, 0.0), 0.0);
}

#[test]
fn the_clamp_keeps_the_pressure_point_inside_the_patch_and_reports_the_move() {
    let s = sole();
    assert_eq!(s.clamp(0.0, 0.0), (0.0, 0.0, false));
    assert_eq!(s.clamp(0.5, 0.0), (0.134, 0.0, true));
    assert_eq!(s.clamp(-0.5, 0.0), (-0.058, 0.0, true));
    assert_eq!(s.clamp(0.0, 0.5), (0.0, 0.034, true));
    assert_eq!(s.clamp(0.0, -0.5), (0.0, -0.034, true));
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-12
}

#[test]
fn the_margin_is_the_nearest_edge() {
    let s = sole();
    assert!(
        close(s.margin(0.0, 0.0), 0.034),
        "the lateral half-width is nearest"
    );
    assert!(close(s.margin(0.0, 0.030), 0.004));
    assert!(close(s.margin(0.0, -0.030), 0.004));
    assert!(
        close(s.margin(-0.05, 0.0), 0.008),
        "the heel is the shorter fore-aft half"
    );
    assert!(
        s.margin(0.2, 0.0) < 0.0,
        "past the toe the travel left is owed, not zero"
    );
}
