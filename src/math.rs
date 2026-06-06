use datapod::{Euler, Point, Quaternion};
use nalgebra::{Matrix3, RowVector3};

pub type Mat3 = Matrix3<f64>;
pub type Vec3 = Point;
pub type Quat = Quaternion;

pub fn vec3(x: f64, y: f64, z: f64) -> Vec3 {
    Point::new(x, y, z)
}

pub fn vec3_zero() -> Vec3 {
    vec3(0.0, 0.0, 0.0)
}

pub fn vec3_x() -> Vec3 {
    vec3(1.0, 0.0, 0.0)
}

pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

pub fn length_squared(v: Vec3) -> f64 {
    dot(v, v)
}

pub fn length(v: Vec3) -> f64 {
    length_squared(v).sqrt()
}

pub fn normalize(v: Vec3) -> Vec3 {
    let n = length(v);
    if n < 1e-20 { vec3_zero() } else { v / n }
}

pub fn negate(v: Vec3) -> Vec3 {
    vec3(-v.x, -v.y, -v.z)
}

pub fn quat_identity() -> Quat {
    Quat::identity()
}

pub fn quat_dot(a: Quat, b: Quat) -> f64 {
    a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
}

pub fn quat_from_xyzw(x: f64, y: f64, z: f64, w: f64) -> Quat {
    Quat::new(w, x, y, z).normalized()
}

pub fn quat_from_axis_angle(axis: Vec3, angle_rad: f64) -> Quat {
    let axis = normalize(axis);
    let half = angle_rad * 0.5;
    let (sin_half, cos_half) = half.sin_cos();
    Quat::new(
        cos_half,
        axis.x * sin_half,
        axis.y * sin_half,
        axis.z * sin_half,
    )
    .normalized()
}

pub fn quat_from_euler_zyx(yaw_rad: f64, pitch_rad: f64, roll_rad: f64) -> Quat {
    Quat::from_euler(Euler::new(roll_rad, pitch_rad, yaw_rad)).normalized()
}

pub fn quat_rotate(q: Quat, point: Vec3) -> Vec3 {
    let mut x = point.x;
    let mut y = point.y;
    let mut z = point.z;
    q.rotate_vector(&mut x, &mut y, &mut z);
    vec3(x, y, z)
}

pub fn quat_slerp(a: Quat, b: Quat, t: f64) -> Quat {
    let mut end = b;
    let mut cos_theta = quat_dot(a, b);
    if cos_theta < 0.0 {
        end = Quat::new(-b.w, -b.x, -b.y, -b.z);
        cos_theta = -cos_theta;
    }
    if cos_theta > 0.9995 {
        let blended = Quat::new(
            a.w + t * (end.w - a.w),
            a.x + t * (end.x - a.x),
            a.y + t * (end.y - a.y),
            a.z + t * (end.z - a.z),
        );
        return blended.normalized();
    }

    let theta = cos_theta.acos();
    let sin_theta = theta.sin();
    let wa = ((1.0 - t) * theta).sin() / sin_theta;
    let wb = (t * theta).sin() / sin_theta;
    Quat::new(
        a.w * wa + end.w * wb,
        a.x * wa + end.x * wb,
        a.y * wa + end.y * wb,
        a.z * wa + end.z * wb,
    )
    .normalized()
}

pub fn quat_to_matrix(q: Quat) -> Mat3 {
    let q = q.normalized();
    let xx = q.x * q.x;
    let yy = q.y * q.y;
    let zz = q.z * q.z;
    let xy = q.x * q.y;
    let xz = q.x * q.z;
    let yz = q.y * q.z;
    let wx = q.w * q.x;
    let wy = q.w * q.y;
    let wz = q.w * q.z;

    Mat3::new(
        1.0 - 2.0 * (yy + zz),
        2.0 * (xy - wz),
        2.0 * (xz + wy),
        2.0 * (xy + wz),
        1.0 - 2.0 * (xx + zz),
        2.0 * (yz - wx),
        2.0 * (xz - wy),
        2.0 * (yz + wx),
        1.0 - 2.0 * (xx + yy),
    )
}

pub fn quat_from_matrix(matrix: Mat3) -> Quat {
    // Element access via `[(row, col)]`.
    let m = &matrix;
    let trace = m[(0, 0)] + m[(1, 1)] + m[(2, 2)];

    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        Quat::new(
            0.25 * s,
            (m[(2, 1)] - m[(1, 2)]) / s,
            (m[(0, 2)] - m[(2, 0)]) / s,
            (m[(1, 0)] - m[(0, 1)]) / s,
        )
    } else if m[(0, 0)] > m[(1, 1)] && m[(0, 0)] > m[(2, 2)] {
        let s = (1.0 + m[(0, 0)] - m[(1, 1)] - m[(2, 2)]).sqrt() * 2.0;
        Quat::new(
            (m[(2, 1)] - m[(1, 2)]) / s,
            0.25 * s,
            (m[(0, 1)] + m[(1, 0)]) / s,
            (m[(0, 2)] + m[(2, 0)]) / s,
        )
    } else if m[(1, 1)] > m[(2, 2)] {
        let s = (1.0 + m[(1, 1)] - m[(0, 0)] - m[(2, 2)]).sqrt() * 2.0;
        Quat::new(
            (m[(0, 2)] - m[(2, 0)]) / s,
            (m[(0, 1)] + m[(1, 0)]) / s,
            0.25 * s,
            (m[(1, 2)] + m[(2, 1)]) / s,
        )
    } else {
        let s = (1.0 + m[(2, 2)] - m[(0, 0)] - m[(1, 1)]).sqrt() * 2.0;
        Quat::new(
            (m[(1, 0)] - m[(0, 1)]) / s,
            (m[(0, 2)] + m[(2, 0)]) / s,
            (m[(1, 2)] + m[(2, 1)]) / s,
            0.25 * s,
        )
    };

    q.normalized()
}

pub fn mat3_identity() -> Mat3 {
    Mat3::identity()
}

pub fn mat3_from_rows(rows: [[f64; 3]; 3]) -> Mat3 {
    Mat3::from_rows(&[
        RowVector3::new(rows[0][0], rows[0][1], rows[0][2]),
        RowVector3::new(rows[1][0], rows[1][1], rows[1][2]),
        RowVector3::new(rows[2][0], rows[2][1], rows[2][2]),
    ])
}

pub fn mat3_transpose(matrix: Mat3) -> Mat3 {
    matrix.transpose()
}

pub fn mat3_mul_vec(matrix: Mat3, vector: Vec3) -> Vec3 {
    let v = matrix * nalgebra::Vector3::new(vector.x, vector.y, vector.z);
    vec3(v.x, v.y, v.z)
}

pub fn mat3_mul_mat(a: Mat3, b: Mat3) -> Mat3 {
    a * b
}

pub fn mat3_scale(matrix: Mat3, scalar: f64) -> Mat3 {
    matrix * scalar
}

pub fn mat3_add(a: Mat3, b: Mat3) -> Mat3 {
    a + b
}
