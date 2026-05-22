use std::{fmt, marker::PhantomData, ops::Mul};

use datapod::{Point, Quaternion};

use crate::math::{
    Mat3, Quat, Vec3, negate, quat_from_axis_angle, quat_from_euler_zyx, quat_from_matrix,
    quat_identity, quat_rotate, quat_to_matrix,
};

#[derive(Clone, Copy, PartialEq)]
pub struct Rotation<To, From, T = f64> {
    pub quat: Quaternion,
    _marker: PhantomData<(To, From, T)>,
}

impl<To, From, T> fmt::Debug for Rotation<To, From, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Rotation")
            .field("quat", &self.quat)
            .finish()
    }
}

impl<To, From, T> Rotation<To, From, T> {
    pub fn new(quat: Quat) -> Self {
        Self {
            quat: quat.normalized(),
            _marker: PhantomData,
        }
    }

    pub fn from_quat(quat: Quat) -> Self {
        Self::new(quat)
    }

    pub fn from_matrix(matrix: crate::math::Mat3) -> Self {
        Self::new(quat_from_matrix(matrix))
    }

    pub fn from_axis_angle(axis: Vec3, angle_rad: f64) -> Self {
        if axis.magnitude().powi(2) < 1e-20 {
            return Self::identity();
        }
        Self::new(quat_from_axis_angle(axis, angle_rad))
    }

    pub fn from_euler_zyx(yaw_rad: f64, pitch_rad: f64, roll_rad: f64) -> Self {
        Self::new(quat_from_euler_zyx(yaw_rad, pitch_rad, roll_rad))
    }

    pub fn identity() -> Self {
        Self::new(quat_identity())
    }

    pub fn inverse(self) -> Rotation<From, To, T> {
        Rotation::new(self.quat.conjugate())
    }

    pub fn apply(&self, point: Point) -> Point {
        quat_rotate(self.quat, point)
    }

    pub fn to_matrix(&self) -> Mat3 {
        quat_to_matrix(self.quat)
    }

    pub fn quaternion(&self) -> Quat {
        self.quat
    }
}

impl<A, B, C, T> Mul<Rotation<B, C, T>> for Rotation<A, B, T> {
    type Output = Rotation<A, C, T>;

    fn mul(self, rhs: Rotation<B, C, T>) -> Self::Output {
        Rotation::new(self.quat * rhs.quat)
    }
}

impl<To, From, T> Mul<Point> for Rotation<To, From, T> {
    type Output = Point;

    fn mul(self, rhs: Point) -> Self::Output {
        self.apply(rhs)
    }
}

#[derive(Clone, Copy, PartialEq)]
pub struct Transform<To, From, T = f64> {
    pub rotation: Rotation<To, From, T>,
    pub translation: Point,
}

impl<To, From, T> fmt::Debug for Transform<To, From, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Transform")
            .field("rotation", &self.rotation)
            .field("translation", &self.translation)
            .finish()
    }
}

impl<To, From, T> Transform<To, From, T> {
    pub fn new(rotation: Rotation<To, From, T>, translation: Point) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn from_qt(quat: Quaternion, translation: Point) -> Self {
        Self::new(Rotation::new(quat), translation)
    }

    pub fn from_rt(matrix: crate::math::Mat3, translation: Point) -> Self {
        Self::new(Rotation::from_matrix(matrix), translation)
    }

    pub fn identity() -> Self {
        Self::new(Rotation::identity(), Point::new(0.0, 0.0, 0.0))
    }

    pub fn inverse(self) -> Transform<From, To, T> {
        let rotation = self.rotation.inverse();
        let translation = negate(rotation.apply(self.translation));
        Transform::new(rotation, translation)
    }

    pub fn apply(&self, point: Point) -> Point {
        self.rotation.apply(point) + self.translation
    }

    pub fn rotation_matrix(&self) -> Mat3 {
        self.rotation.to_matrix()
    }
}

impl<A, B, C, T> Mul<Transform<B, C, T>> for Transform<A, B, T> {
    type Output = Transform<A, C, T>;

    fn mul(self, rhs: Transform<B, C, T>) -> Self::Output {
        let rotated_translation = self.rotation.apply(rhs.translation);
        let rotation = Rotation::new(self.rotation.quat * rhs.rotation.quat);
        Transform::new(rotation, rotated_translation + self.translation)
    }
}

impl<To, From, T> Mul<Point> for Transform<To, From, T> {
    type Output = Point;

    fn mul(self, rhs: Point) -> Self::Output {
        self.apply(rhs)
    }
}
