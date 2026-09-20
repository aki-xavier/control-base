// contact_footprint.rs — the patch a friction contact's pressure may act in, held to its own arithmetic.
//
// Why the patch is worth its own file: this geometry is the ONLY thing bounding a standing machine's
// balance authority — the moment a foot can still add is its normal load times the travel left inside
// the patch — so a sign error here is a machine that cannot be held up, and every later consumer (the
// support region, the load split, the anti-windup guard) inherits it.

use control_base::plant::Footprint;
use control_math::vec3::Vec3;

/// A sole stated the way a machine declares one: the ground acts below and ahead of the ankle-roll
/// link's origin, and the plate runs further forward than back — the asymmetry is what makes a swapped
/// sign visible in an answer rather than invisible.
fn sole() -> Footprint {
    Footprint {
        offset: Vec3::new(0.038, 0.0, -0.035),
        back: 0.058,
        front: 0.134,
        half_w: 0.034,
    }
}

/// A patch of no extent has one pressure point, its centre, which is the whole content of "a point
/// contact can hold no moment".
#[test]
fn a_point_contact_holds_no_moment() {
    let p = Footprint::POINT;
    assert!(!p.clamp(0.0, 0.0).2);
    assert_eq!(p.clamp(0.01, -0.02), (0.0, 0.0, true));
    assert_eq!(p.margin(0.0, 0.0), 0.0);
}

/// The clamp is the command's own saturation, so it has to say BOTH where the pressure point ends up and
/// that it had to be moved — a caller that cannot see the move has no anti-windup.
#[test]
fn the_clamp_keeps_the_pressure_point_inside_the_patch_and_reports_the_move() {
    let s = sole();
    assert_eq!(s.clamp(0.0, 0.0), (0.0, 0.0, false));
    assert_eq!(s.clamp(0.5, 0.0), (0.134, 0.0, true));
    assert_eq!(s.clamp(-0.5, 0.0), (-0.058, 0.0, true));
    assert_eq!(s.clamp(0.0, 0.5), (0.0, 0.034, true));
    assert_eq!(s.clamp(0.0, -0.5), (0.0, -0.034, true));
}

/// A patch is a geometry, so its readings are compared to a tolerance: the margin is a DIFFERENCE of
/// published numbers, and comparing it exactly would be testing a decimal expansion rather than the patch.
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-12
}

/// The margin is read as an authority, so it is the nearest edge and not a per-axis pair: a pressure
/// point already outside the patch reports how far, rather than saturating at zero.
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
