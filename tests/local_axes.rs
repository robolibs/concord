use datapod::{Point, mat::Matrix};

use concord::earth::{r_enu_from_ecf, r_ned_from_ecf};

fn mul_mat(matrix: Matrix<f64, 3, 3>, vector: Point) -> Point {
    let m = matrix.as_rows();
    Point::new(
        m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z,
        m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z,
        m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z,
    )
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
