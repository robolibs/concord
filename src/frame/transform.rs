use std::{fmt, marker::PhantomData, ops::Mul};

use glam::{DMat3, DQuat, DVec3, EulerRot};

#[derive(Clone, Copy, PartialEq)]
pub struct Rotation<To, From, T = f64> {
    pub quat: DQuat,
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
    pub fn new(quat: DQuat) -> Self {
        Self {
            quat: quat.normalize(),
            _marker: PhantomData,
        }
    }

    pub fn from_quat(quat: DQuat) -> Self {
        Self::new(quat)
    }

    pub fn from_matrix(matrix: DMat3) -> Self {
        Self::new(DQuat::from_mat3(&matrix))
    }

    pub fn from_axis_angle(axis: DVec3, angle_rad: f64) -> Self {
        if axis.length_squared() < 1e-20 {
            return Self::identity();
        }
        Self::new(DQuat::from_axis_angle(axis.normalize(), angle_rad))
    }

    pub fn from_euler_zyx(yaw_rad: f64, pitch_rad: f64, roll_rad: f64) -> Self {
        Self::new(DQuat::from_euler(
            EulerRot::ZYX,
            yaw_rad,
            pitch_rad,
            roll_rad,
        ))
    }

    pub fn identity() -> Self {
        Self::new(DQuat::IDENTITY)
    }

    pub fn inverse(self) -> Rotation<From, To, T> {
        Rotation::new(self.quat.conjugate())
    }

    pub fn apply(&self, point: DVec3) -> DVec3 {
        self.quat * point
    }

    pub fn to_matrix(&self) -> DMat3 {
        DMat3::from_quat(self.quat)
    }

    pub fn quaternion(&self) -> DQuat {
        self.quat
    }
}

impl<A, B, C, T> Mul<Rotation<B, C, T>> for Rotation<A, B, T> {
    type Output = Rotation<A, C, T>;

    fn mul(self, rhs: Rotation<B, C, T>) -> Self::Output {
        Rotation::new(self.quat * rhs.quat)
    }
}

impl<To, From, T> Mul<DVec3> for Rotation<To, From, T> {
    type Output = DVec3;

    fn mul(self, rhs: DVec3) -> Self::Output {
        self.apply(rhs)
    }
}

#[derive(Clone, Copy, PartialEq)]
pub struct Transform<To, From, T = f64> {
    pub rotation: Rotation<To, From, T>,
    pub translation: DVec3,
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
    pub fn new(rotation: Rotation<To, From, T>, translation: DVec3) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn from_qt(quat: DQuat, translation: DVec3) -> Self {
        Self::new(Rotation::new(quat), translation)
    }

    pub fn from_rt(matrix: DMat3, translation: DVec3) -> Self {
        Self::new(Rotation::from_matrix(matrix), translation)
    }

    pub fn identity() -> Self {
        Self::new(Rotation::identity(), DVec3::ZERO)
    }

    pub fn inverse(self) -> Transform<From, To, T> {
        let rotation = self.rotation.inverse();
        let translation = -(rotation.apply(self.translation));
        Transform::new(rotation, translation)
    }

    pub fn apply(&self, point: DVec3) -> DVec3 {
        self.rotation.apply(point) + self.translation
    }

    pub fn rotation_matrix(&self) -> DMat3 {
        self.rotation.to_matrix()
    }
}

impl<A, B, C, T> Mul<Transform<B, C, T>> for Transform<A, B, T> {
    type Output = Transform<A, C, T>;

    fn mul(self, rhs: Transform<B, C, T>) -> Self::Output {
        let rotated_translation = self.rotation.quat * rhs.translation;
        let rotation = Rotation::new(self.rotation.quat * rhs.rotation.quat);
        Transform::new(rotation, rotated_translation + self.translation)
    }
}

impl<To, From, T> Mul<DVec3> for Transform<To, From, T> {
    type Output = DVec3;

    fn mul(self, rhs: DVec3) -> Self::Output {
        self.apply(rhs)
    }
}
