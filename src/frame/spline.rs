use crate::frame::{
    Rotation, Transform, interpolate_rotation, interpolate_transform, is_approx_rotation,
    is_approx_transform,
};

#[derive(Debug, Clone)]
pub struct RotationSpline<To, From, T = f64> {
    points: Vec<Rotation<To, From, T>>,
    built: bool,
}

impl<To, From, T> Default for RotationSpline<To, From, T> {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            built: false,
        }
    }
}

impl<To, From, T> RotationSpline<To, From, T> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_points(points: Vec<Rotation<To, From, T>>) -> Self {
        let mut spline = Self {
            points,
            built: false,
        };
        if spline.points.len() >= 2 {
            spline.build();
        }
        spline
    }

    pub fn add_point(&mut self, rot: Rotation<To, From, T>) {
        self.points.push(rot);
        self.built = false;
    }

    pub fn clear(&mut self) {
        self.points.clear();
        self.built = false;
    }

    pub fn size(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn is_built(&self) -> bool {
        self.built
    }

    pub fn build(&mut self) {
        self.built = self.points.len() >= 2;
    }

    pub fn evaluate(&self, t: f64) -> Rotation<To, From, T> {
        if !self.built || self.points.is_empty() {
            return Rotation::identity();
        }
        let (idx, local_t) = segment_parameter(self.points.len(), t);
        interpolate_rotation(
            rotation_copy(&self.points[idx]),
            rotation_copy(&self.points[idx + 1]),
            local_t,
        )
    }

    pub fn evaluate_normalized(&self, u: f64) -> Rotation<To, From, T> {
        if !self.built || self.points.is_empty() {
            return Rotation::identity();
        }
        let max_t = (self.points.len() - 1) as f64;
        self.evaluate(u.clamp(0.0, 1.0) * max_t)
    }

    pub fn point(&self, i: usize) -> Option<&Rotation<To, From, T>> {
        self.points.get(i)
    }
}

#[derive(Debug, Clone)]
pub struct TransformSpline<To, From, T = f64> {
    points: Vec<Transform<To, From, T>>,
    built: bool,
}

impl<To, From, T> Default for TransformSpline<To, From, T> {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            built: false,
        }
    }
}

impl<To, From, T> TransformSpline<To, From, T> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_points(points: Vec<Transform<To, From, T>>) -> Self {
        let mut spline = Self {
            points,
            built: false,
        };
        if spline.points.len() >= 2 {
            spline.build();
        }
        spline
    }

    pub fn add_point(&mut self, tf: Transform<To, From, T>) {
        self.points.push(tf);
        self.built = false;
    }

    pub fn clear(&mut self) {
        self.points.clear();
        self.built = false;
    }

    pub fn size(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn is_built(&self) -> bool {
        self.built
    }

    pub fn build(&mut self) {
        self.built = self.points.len() >= 2;
    }

    pub fn evaluate(&self, t: f64) -> Transform<To, From, T> {
        if !self.built || self.points.is_empty() {
            return Transform::identity();
        }
        let (idx, local_t) = segment_parameter(self.points.len(), t);
        interpolate_transform(
            transform_copy(&self.points[idx]),
            transform_copy(&self.points[idx + 1]),
            local_t,
        )
    }

    pub fn evaluate_normalized(&self, u: f64) -> Transform<To, From, T> {
        if !self.built || self.points.is_empty() {
            return Transform::identity();
        }
        let max_t = (self.points.len() - 1) as f64;
        self.evaluate(u.clamp(0.0, 1.0) * max_t)
    }

    pub fn point(&self, i: usize) -> Option<&Transform<To, From, T>> {
        self.points.get(i)
    }
}

#[derive(Debug, Clone)]
pub struct TimedRotationSpline<To, From, T = f64> {
    times: Vec<f64>,
    points: Vec<Rotation<To, From, T>>,
    built: bool,
}

impl<To, From, T> Default for TimedRotationSpline<To, From, T> {
    fn default() -> Self {
        Self {
            times: Vec::new(),
            points: Vec::new(),
            built: false,
        }
    }
}

impl<To, From, T> TimedRotationSpline<To, From, T> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_point(&mut self, time: f64, rot: Rotation<To, From, T>) {
        let idx = self.times.partition_point(|&t| t < time);
        self.times.insert(idx, time);
        self.points.insert(idx, rot);
        self.built = false;
    }

    pub fn clear(&mut self) {
        self.times.clear();
        self.points.clear();
        self.built = false;
    }

    pub fn size(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn is_built(&self) -> bool {
        self.built
    }

    pub fn time_range(&self) -> Option<(f64, f64)> {
        Some((*self.times.first()?, *self.times.last()?))
    }

    pub fn can_evaluate(&self, time: f64) -> bool {
        self.time_range()
            .map(|(start, end)| self.points.len() >= 2 && time >= start && time <= end)
            .unwrap_or(false)
    }

    pub fn build(&mut self) {
        self.built = self.points.len() >= 2;
    }

    pub fn evaluate_at(&self, time: f64) -> Rotation<To, From, T> {
        if !self.built || self.points.len() < 2 {
            return self
                .points
                .first()
                .map(rotation_copy)
                .unwrap_or_else(Rotation::identity);
        }

        let time = clamp_time(&self.times, time);
        let idx = self.times.partition_point(|&t| t < time);
        if idx < self.times.len() && (self.times[idx] - time).abs() < 1e-12 {
            return rotation_copy(&self.points[idx]);
        }
        let upper = idx.min(self.times.len() - 1).max(1);
        let lower = upper - 1;
        let t0 = self.times[lower];
        let t1 = self.times[upper];
        let local_t = normalized_segment_time(t0, t1, time);
        interpolate_rotation(
            rotation_copy(&self.points[lower]),
            rotation_copy(&self.points[upper]),
            local_t,
        )
    }
}

#[derive(Debug, Clone)]
pub struct TimedTransformSpline<To, From, T = f64> {
    times: Vec<f64>,
    points: Vec<Transform<To, From, T>>,
    built: bool,
}

impl<To, From, T> Default for TimedTransformSpline<To, From, T> {
    fn default() -> Self {
        Self {
            times: Vec::new(),
            points: Vec::new(),
            built: false,
        }
    }
}

impl<To, From, T> TimedTransformSpline<To, From, T> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_point(&mut self, time: f64, tf: Transform<To, From, T>) {
        let idx = self.times.partition_point(|&t| t < time);
        self.times.insert(idx, time);
        self.points.insert(idx, tf);
        self.built = false;
    }

    pub fn clear(&mut self) {
        self.times.clear();
        self.points.clear();
        self.built = false;
    }

    pub fn size(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn is_built(&self) -> bool {
        self.built
    }

    pub fn time_range(&self) -> Option<(f64, f64)> {
        Some((*self.times.first()?, *self.times.last()?))
    }

    pub fn can_evaluate(&self, time: f64) -> bool {
        self.time_range()
            .map(|(start, end)| self.points.len() >= 2 && time >= start && time <= end)
            .unwrap_or(false)
    }

    pub fn build(&mut self) {
        self.built = self.points.len() >= 2;
    }

    pub fn evaluate_at(&self, time: f64) -> Transform<To, From, T> {
        if !self.built || self.points.len() < 2 {
            return self
                .points
                .first()
                .map(transform_copy)
                .unwrap_or_else(Transform::identity);
        }

        let time = clamp_time(&self.times, time);
        let idx = self.times.partition_point(|&t| t < time);
        if idx < self.times.len() && (self.times[idx] - time).abs() < 1e-12 {
            return transform_copy(&self.points[idx]);
        }
        let upper = idx.min(self.times.len() - 1).max(1);
        let lower = upper - 1;
        let t0 = self.times[lower];
        let t1 = self.times[upper];
        let local_t = normalized_segment_time(t0, t1, time);
        interpolate_transform(
            transform_copy(&self.points[lower]),
            transform_copy(&self.points[upper]),
            local_t,
        )
    }
}

pub fn sample_rotation_spline<To, From, T>(
    spline: &RotationSpline<To, From, T>,
    sample_count: usize,
) -> Vec<Rotation<To, From, T>> {
    if sample_count == 0 {
        return Vec::new();
    }
    if sample_count == 1 {
        return vec![spline.evaluate_normalized(0.0)];
    }
    (0..sample_count)
        .map(|i| spline.evaluate_normalized(i as f64 / (sample_count - 1) as f64))
        .collect()
}

pub fn sample_transform_spline<To, From, T>(
    spline: &TransformSpline<To, From, T>,
    sample_count: usize,
) -> Vec<Transform<To, From, T>> {
    if sample_count == 0 {
        return Vec::new();
    }
    if sample_count == 1 {
        return vec![spline.evaluate_normalized(0.0)];
    }
    (0..sample_count)
        .map(|i| spline.evaluate_normalized(i as f64 / (sample_count - 1) as f64))
        .collect()
}

pub fn is_approx_rotation_spline<To, From, T>(
    left: &RotationSpline<To, From, T>,
    right: &RotationSpline<To, From, T>,
    eps: f64,
) -> bool {
    left.size() == right.size()
        && left
            .points
            .iter()
            .zip(&right.points)
            .all(|(a, b)| is_approx_rotation(rotation_copy(a), rotation_copy(b), eps))
}

pub fn is_approx_transform_spline<To, From, T>(
    left: &TransformSpline<To, From, T>,
    right: &TransformSpline<To, From, T>,
    eps: f64,
) -> bool {
    left.size() == right.size()
        && left
            .points
            .iter()
            .zip(&right.points)
            .all(|(a, b)| is_approx_transform(transform_copy(a), transform_copy(b), eps))
}

fn segment_parameter(point_count: usize, t: f64) -> (usize, f64) {
    let max_t = (point_count - 1) as f64;
    let t = t.clamp(0.0, max_t);
    let idx = t.floor() as usize;
    if idx >= point_count - 1 {
        return (point_count - 2, 1.0);
    }
    (idx, t - idx as f64)
}

fn clamp_time(times: &[f64], time: f64) -> f64 {
    time.clamp(times[0], times[times.len() - 1])
}

fn normalized_segment_time(t0: f64, t1: f64, t: f64) -> f64 {
    let dt = t1 - t0;
    if dt.abs() < 1e-12 {
        0.0
    } else {
        ((t - t0) / dt).clamp(0.0, 1.0)
    }
}

fn rotation_copy<To, From, T>(rotation: &Rotation<To, From, T>) -> Rotation<To, From, T> {
    Rotation::from_quat(rotation.quaternion())
}

fn transform_copy<To, From, T>(transform: &Transform<To, From, T>) -> Transform<To, From, T> {
    Transform::new(rotation_copy(&transform.rotation), transform.translation)
}
