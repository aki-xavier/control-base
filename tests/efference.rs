// efference.rs — the command against the sensor read, and the difference between them: engine-free,
// and unconditional — whatever gate there is belongs elsewhere.

use control_base::efference::Efference;

#[test]
fn the_residual_is_the_reading_the_machine_did_not_do_itself() {
    let mut e = Efference::new();
    // commanded == read cancels to zero, so nothing may fire on it; what the world adds is the
    // residual, and the share is that over the reading
    let r = e.observe(&[163.0, 163.0], &[163.0, 163.0]);
    assert_eq!(r, 0.0);
    assert_eq!(e.residual(0), 0.0);
    assert_eq!(e.observe(&[163.0, 163.0], &[263.0, 163.0]), 100.0);
    assert_eq!(e.residual(0), 100.0);
    assert_eq!(e.residual(1), 0.0);
    assert!((e.external_share(0) - 100.0 / 263.0).abs() < 1e-12);
    assert_eq!(e.external_share(1), 0.0);
    // a reading that is entirely the machine's own doing has a share of zero, whatever the force
    let mut e2 = Efference::new();
    e2.observe(&[500.0, 500.0], &[500.0, 500.0]);
    assert_eq!(e2.external_share(0), 0.0);
    assert_eq!(e2.residual(0), 0.0);
    // a reading of zero with nothing commanded answers zero rather than 0/0
    let mut e3 = Efference::new();
    e3.observe(&[0.0], &[0.0]);
    assert_eq!(e3.external_share(0), 0.0);
    // there is no off switch: a fresh copy has simply not been handed a pair yet
    let e4 = Efference::new();
    assert_eq!(e4.samples, 0);
    assert_eq!(e4.residual(0), 0.0);
    assert!(e4.report().contains("0 samples"));
    let mut e5 = Efference::new();
    e5.observe(&[10.0], &[12.0]);
    assert_eq!(e5.samples, 1);
    assert_eq!(e5.residual(0), 2.0);
    assert!(e3.report().contains("residual"));
}
