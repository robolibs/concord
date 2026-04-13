use datapod::Point;

use crate::frame::{Rotation, Transform};
use crate::math::{
    Mat3, mat3_add, mat3_from_rows, mat3_identity, mat3_mul_mat, mat3_mul_vec, mat3_scale,
    quat_dot, quat_from_xyzw, quat_slerp, vec3, vec3_x, vec3_zero,
};

pub type RotationTangent = Point;
pub type TransformTangent = [f64; 6];

fn skew(v: Point) -> Mat3 {
    mat3_from_rows([[0.0, -v.z, v.y], [v.z, 0.0, -v.x], [-v.y, v.x, 0.0]])
}

fn so3_left_jacobian(omega: Point) -> Mat3 {
    let theta = omega.magnitude();
    let omega_hat = skew(omega);
    let omega_hat2 = mat3_mul_mat(omega_hat, omega_hat);
    if theta < 1e-12 {
        return mat3_add(
            mat3_add(mat3_identity(), mat3_scale(omega_hat, 0.5)),
            mat3_scale(omega_hat2, 1.0 / 6.0),
        );
    }
    let b = (1.0 - theta.cos()) / (theta * theta);
    let c = (theta - theta.sin()) / (theta * theta * theta);
    mat3_add(
        mat3_add(mat3_identity(), mat3_scale(omega_hat, b)),
        mat3_scale(omega_hat2, c),
    )
}

fn so3_left_jacobian_inverse(omega: Point) -> Mat3 {
    let theta = omega.magnitude();
    let omega_hat = skew(omega);
    let omega_hat2 = mat3_mul_mat(omega_hat, omega_hat);
    if theta < 1e-12 {
        return mat3_add(
            mat3_add(mat3_identity(), mat3_scale(omega_hat, -0.5)),
            mat3_scale(omega_hat2, 1.0 / 12.0),
        );
    }
    let coeff = 1.0 / (theta * theta) - (1.0 + theta.cos()) / (2.0 * theta * theta.sin());
    mat3_add(
        mat3_add(mat3_identity(), mat3_scale(omega_hat, -0.5)),
        mat3_scale(omega_hat2, coeff),
    )
}

pub fn exp<To, From, T>(tangent: impl Into<LieInput>) -> LieOutput<To, From, T> {
    match tangent.into() {
        LieInput::Rotation(omega) => LieOutput::Rotation(rotation_exp::<To, From, T>(omega)),
        LieInput::Transform(twist) => LieOutput::Transform(transform_exp::<To, From, T>(twist)),
    }
}

pub enum LieInput {
    Rotation(RotationTangent),
    Transform(TransformTangent),
}

pub enum LieOutput<To, From, T = f64> {
    Rotation(Rotation<To, From, T>),
    Transform(Transform<To, From, T>),
}

impl From<RotationTangent> for LieInput {
    fn from(value: RotationTangent) -> Self {
        Self::Rotation(value)
    }
}

impl From<TransformTangent> for LieInput {
    fn from(value: TransformTangent) -> Self {
        Self::Transform(value)
    }
}

pub fn rotation_exp<To, From, T>(omega: RotationTangent) -> Rotation<To, From, T> {
    let theta = omega.magnitude();
    if theta < 1e-12 {
        Rotation::identity()
    } else {
        Rotation::from_axis_angle(omega / theta, theta)
    }
}

pub fn rotation_log<To, From, T>(rotation: Rotation<To, From, T>) -> RotationTangent {
    let q = rotation.quaternion().normalized();
    let vec = vec3(q.x, q.y, q.z);
    let sin_half = vec.magnitude();
    if sin_half < 1e-12 {
        return vec3_zero();
    }
    let axis = vec / sin_half;
    let angle = 2.0 * sin_half.atan2(q.w);
    axis * angle
}

pub fn transform_exp<To, From, T>(twist: TransformTangent) -> Transform<To, From, T> {
    let v = vec3(twist[0], twist[1], twist[2]);
    let omega = vec3(twist[3], twist[4], twist[5]);
    let rotation = rotation_exp::<To, From, T>(omega);
    let translation = if omega.magnitude() < 1e-12 {
        v
    } else {
        mat3_mul_vec(so3_left_jacobian(omega), v)
    };
    Transform::new(rotation, translation)
}

pub fn transform_log<To, From, T>(transform: Transform<To, From, T>) -> TransformTangent {
    let omega = rotation_log(transform.rotation);
    let v = if omega.magnitude() < 1e-12 {
        transform.translation
    } else {
        mat3_mul_vec(so3_left_jacobian_inverse(omega), transform.translation)
    };
    [v.x, v.y, v.z, omega.x, omega.y, omega.z]
}

pub fn log_rotation<To, From, T>(rotation: Rotation<To, From, T>) -> RotationTangent {
    rotation_log(rotation)
}

pub fn log_transform<To, From, T>(transform: Transform<To, From, T>) -> TransformTangent {
    transform_log(transform)
}

pub fn slerp<To, From, T>(
    a: Rotation<To, From, T>,
    b: Rotation<To, From, T>,
    t: f64,
) -> Rotation<To, From, T> {
    Rotation::from_quat(quat_slerp(a.quaternion(), b.quaternion(), t))
}

pub fn interpolate_rotation<To, From, T>(
    a: Rotation<To, From, T>,
    b: Rotation<To, From, T>,
    t: f64,
) -> Rotation<To, From, T> {
    slerp(a, b, t)
}

pub fn interpolate_transform<To, From, T>(
    a: Transform<To, From, T>,
    b: Transform<To, From, T>,
    t: f64,
) -> Transform<To, From, T> {
    Transform::new(
        slerp(a.rotation, b.rotation, t),
        a.translation + (b.translation - a.translation) * t,
    )
}

pub fn average_two_rotation<To, From, T>(
    a: Rotation<To, From, T>,
    b: Rotation<To, From, T>,
) -> Rotation<To, From, T> {
    slerp(a, b, 0.5)
}

pub fn average_two_transform<To, From, T>(
    a: Transform<To, From, T>,
    b: Transform<To, From, T>,
) -> Transform<To, From, T> {
    interpolate_transform(a, b, 0.5)
}

pub fn average_rotation<To, From, T>(
    rotations: &[Rotation<To, From, T>],
) -> Option<Rotation<To, From, T>> {
    match rotations {
        [] => None,
        [one] => Some(Rotation::from_quat(one.quaternion())),
        many => {
            let mut acc = many[0].quaternion();
            for rotation in &many[1..] {
                let q = rotation.quaternion();
                if quat_dot(acc, q) < 0.0 {
                    acc = quat_from_xyzw(acc.x - q.x, acc.y - q.y, acc.z - q.z, acc.w - q.w);
                } else {
                    acc = quat_from_xyzw(acc.x + q.x, acc.y + q.y, acc.z + q.z, acc.w + q.w);
                }
            }
            Some(Rotation::from_quat(acc.normalized()))
        }
    }
}

pub fn angle<To, From, T>(rotation: Rotation<To, From, T>) -> f64 {
    rotation_log(rotation).magnitude()
}

pub fn axis<To, From, T>(rotation: Rotation<To, From, T>) -> RotationTangent {
    let omega = rotation_log(rotation);
    let theta = omega.magnitude();
    if theta < 1e-12 { vec3_x() } else { omega / theta }
}

pub fn is_identity_rotation<To, From, T>(rotation: Rotation<To, From, T>, eps: f64) -> bool {
    angle(rotation) < eps
}

pub fn is_identity_transform<To, From, T>(transform: Transform<To, From, T>, eps: f64) -> bool {
    is_identity_rotation(transform.rotation, eps) && transform.translation.magnitude() < eps
}

pub fn is_approx_rotation<To, From, T>(
    a: Rotation<To, From, T>,
    b: Rotation<To, From, T>,
    eps: f64,
) -> bool {
    let qa = a.quaternion();
    let qb = b.quaternion();
    let same = vec3(qa.x - qb.x, qa.y - qb.y, qa.z - qb.z).magnitude() < eps
        && (qa.w - qb.w).abs() < eps;
    let opposite = vec3(qa.x + qb.x, qa.y + qb.y, qa.z + qb.z).magnitude() < eps
        && (qa.w + qb.w).abs() < eps;
    same || opposite
}

pub fn is_approx_transform<To, From, T>(
    a: Transform<To, From, T>,
    b: Transform<To, From, T>,
    eps: f64,
) -> bool {
    is_approx_rotation(a.rotation, b.rotation, eps)
        && (a.translation - b.translation).magnitude() < eps
}
