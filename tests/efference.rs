use control_base::efference::Efference;

#[test]
fn the_residual_is_the_reading_the_machine_did_not_do_itself() {
    let mut e = Efference::new();
    let r = e.observe(&[163.0, 163.0], &[163.0, 163.0]);
    assert_eq!(r, 0.0);
    assert_eq!(e.residual(0), 0.0);
    assert_eq!(e.observe(&[163.0, 163.0], &[263.0, 163.0]), 100.0);
    assert_eq!(e.residual(0), 100.0);
    assert_eq!(e.residual(1), 0.0);
    assert!((e.external_share(0) - 100.0 / 263.0).abs() < 1e-12);
    assert_eq!(e.external_share(1), 0.0);
    let mut e2 = Efference::new();
    e2.observe(&[500.0, 500.0], &[500.0, 500.0]);
    assert_eq!(e2.external_share(0), 0.0);
    assert_eq!(e2.residual(0), 0.0);
    let mut e3 = Efference::new();
    e3.observe(&[0.0], &[0.0]);
    assert_eq!(e3.external_share(0), 0.0);
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
