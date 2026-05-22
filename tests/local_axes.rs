use datapod::Point;

use concord::earth::{r_enu_from_ecf, r_ned_from_ecf};
use concord::math::Mat3;

fn mul_mat(matrix: Mat3, vector: Point) -> Point {
    let v = nalgebra::Vector3::new(vector.x, vector.y, vector.z);
    let r = matrix * v;
    Point::new(r[0], r[1], r[2])
}

fn approx_vec(a: Point, b: Point, eps: f64) {
    assert!((a - b).magnitude() < eps, "a={a:?} b={b:?}");
}

#[test]
fn enu_axes_at_equator_prime_meridian() {
    let r = r_enu_from_ecf(0.0, 0.0);

    approx_vec(mul_mat(r, Point::new(1.0, 0.0, 0.0)), Point::new(0.0, 0.0, 1.0), 1e-12);
    approx_vec(mul_mat(r, Point::new(0.0, 1.0, 0.0)), Point::new(1.0, 0.0, 0.0), 1e-12);
    approx_vec(mul_mat(r, Point::new(0.0, 0.0, 1.0)), Point::new(0.0, 1.0, 0.0), 1e-12);
}

#[test]
fn ned_axes_at_equator_prime_meridian() {
    let r = r_ned_from_ecf(0.0, 0.0);

    approx_vec(mul_mat(r, Point::new(1.0, 0.0, 0.0)), Point::new(0.0, 0.0, -1.0), 1e-12);
    approx_vec(mul_mat(r, Point::new(0.0, 1.0, 0.0)), Point::new(0.0, 1.0, 0.0), 1e-12);
    approx_vec(mul_mat(r, Point::new(0.0, 0.0, 1.0)), Point::new(1.0, 0.0, 0.0), 1e-12);
}
