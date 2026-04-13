use concord::earth::{Wgs, to_utm, utm_to_wgs};

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!(
        (a - b).abs() < eps,
        "left={a}, right={b}, diff={}",
        (a - b).abs()
    );
}

#[test]
fn wgs_to_utm_roundtrip() {
    let paris = Wgs::new(48.8566, 2.3522, 35.0);
    let utm = to_utm(paris).expect("valid utm");
    let back = utm_to_wgs(utm).expect("valid roundtrip");

    approx_eq(back.latitude, paris.latitude, 1e-8);
    approx_eq(back.longitude, paris.longitude, 1e-8);
    approx_eq(back.altitude, paris.altitude, 1e-6);
}

#[test]
fn utm_out_of_bounds_errors() {
    let out = to_utm(Wgs::new(85.0, 0.0, 0.0));
    assert!(out.is_err());
}
