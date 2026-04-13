use glam::DVec3;

use concord::earth::{r_enu_from_ecf, r_ned_from_ecf};

fn approx_vec(a: DVec3, b: DVec3, eps: f64) {
    assert!((a - b).length() < eps, "a={a:?} b={b:?}");
}

#[test]
fn enu_axes_at_equator_prime_meridian() {
    let r = r_enu_from_ecf(0.0, 0.0);

    approx_vec(r * DVec3::X, DVec3::new(0.0, 0.0, 1.0), 1e-12);
    approx_vec(r * DVec3::Y, DVec3::new(1.0, 0.0, 0.0), 1e-12);
    approx_vec(r * DVec3::Z, DVec3::new(0.0, 1.0, 0.0), 1e-12);
}

#[test]
fn ned_axes_at_equator_prime_meridian() {
    let r = r_ned_from_ecf(0.0, 0.0);

    approx_vec(r * DVec3::X, DVec3::new(0.0, 0.0, -1.0), 1e-12);
    approx_vec(r * DVec3::Y, DVec3::new(0.0, 1.0, 0.0), 1e-12);
    approx_vec(r * DVec3::Z, DVec3::new(1.0, 0.0, 0.0), 1e-12);
}
