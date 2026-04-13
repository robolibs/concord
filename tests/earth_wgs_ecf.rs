use concord::earth::{Wgs, to_ecf, to_wgs, to_wgs_optimized, wgs84};

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!(
        (a - b).abs() < eps,
        "left={a}, right={b}, diff={}",
        (a - b).abs()
    );
}

#[test]
fn wgs_to_ecf_to_wgs_roundtrip() {
    let original = Wgs::new(48.8566, 2.3522, 35.0);
    let ecf = to_ecf(original);
    let back = to_wgs(ecf);

    approx_eq(back.latitude, original.latitude, 1e-9);
    approx_eq(back.longitude, original.longitude, 1e-9);
    approx_eq(back.altitude, original.altitude, 1e-5);
}

#[test]
fn ecf_known_values_at_equator_prime_meridian() {
    let ecf = to_ecf(Wgs::new(0.0, 0.0, 0.0));

    approx_eq(ecf.x, wgs84::A_M, 1.0);
    approx_eq(ecf.y, 0.0, 1.0);
    approx_eq(ecf.z, 0.0, 1.0);
}

#[test]
fn ecf_supports_vector_magnitude() {
    let ecf = concord::Ecf::new(1000.0, 2000.0, 3000.0);
    assert!(ecf.magnitude() > 0.0);
}

#[test]
fn to_wgs_optimized_basic_roundtrip() {
    let original = Wgs::new(48.8566, 2.3522, 35.0);
    let ecf = to_ecf(original);
    let back = to_wgs_optimized(ecf, 1e-12);

    approx_eq(back.latitude, original.latitude, 1e-9);
    approx_eq(back.longitude, original.longitude, 1e-9);
    approx_eq(back.altitude, original.altitude, 1e-5);
}

#[test]
fn to_wgs_optimized_handles_extreme_altitudes() {
    for altitude in [-100.0, 0.0, 100.0, 10_000.0, 100_000.0, 35_786_000.0] {
        let original = Wgs::new(45.0, 90.0, altitude);
        let ecf = to_ecf(original);
        let back = to_wgs_optimized(ecf, 1e-12);

        approx_eq(back.latitude, original.latitude, 1e-8);
        approx_eq(back.longitude, original.longitude, 1e-8);

        let alt_err = (back.altitude - original.altitude).abs();
        let rel_err = if altitude != 0.0 {
            alt_err / altitude.abs()
        } else {
            alt_err
        };
        assert!(rel_err < 1e-8 || alt_err < 1e-3);
    }
}
