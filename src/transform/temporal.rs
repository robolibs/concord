use std::collections::{HashMap, HashSet};

use glam::{DQuat, DVec3};
use graphix::vertex::VertexId;

use crate::{
    frame::Transform,
    transform::{FrameGraph, FrameInfo, GenericTransform},
};

pub fn slerp(q0: DQuat, q1: DQuat, t: f64) -> DQuat {
    q0.slerp(q1, t)
}

pub fn lerp(p0: DVec3, p1: DVec3, t: f64) -> DVec3 {
    p0 + (p1 - p0) * t
}

pub fn interpolate(tf0: GenericTransform, tf1: GenericTransform, t: f64) -> GenericTransform {
    GenericTransform::new(
        slerp(tf0.rotation, tf1.rotation, t),
        lerp(tf0.translation, tf1.translation, t),
    )
}

#[derive(Debug, Clone)]
pub struct TimedTransformBuffer<const BUFFER_SIZE: usize = 100> {
    timestamps: Vec<f64>,
    values: Vec<GenericTransform>,
    max_size: usize,
}

impl<const BUFFER_SIZE: usize> Default for TimedTransformBuffer<BUFFER_SIZE> {
    fn default() -> Self {
        Self {
            timestamps: Vec::new(),
            values: Vec::new(),
            max_size: BUFFER_SIZE,
        }
    }
}

impl<const BUFFER_SIZE: usize> TimedTransformBuffer<BUFFER_SIZE> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_max_size(max_size: usize) -> Self {
        Self {
            max_size,
            ..Self::default()
        }
    }

    pub fn add(&mut self, timestamp: f64, tf: GenericTransform) {
        self.timestamps.push(timestamp);
        self.values.push(tf);
        while self.timestamps.len() > self.max_size {
            self.timestamps.remove(0);
            self.values.remove(0);
        }
        self.sort_by_time();
    }

    pub fn lookup(&self, timestamp: f64) -> Option<GenericTransform> {
        if self.timestamps.is_empty() {
            return None;
        }
        if self.timestamps.len() == 1 {
            return ((self.timestamps[0] - timestamp).abs() < 1e-10).then_some(self.values[0]);
        }
        if !self.in_range(timestamp) {
            return None;
        }

        let idx = self.timestamps.partition_point(|&t| t < timestamp);
        if idx < self.timestamps.len() && (self.timestamps[idx] - timestamp).abs() < 1e-10 {
            return Some(self.values[idx]);
        }
        if idx == 0 || idx >= self.timestamps.len() {
            return None;
        }

        let before = idx - 1;
        let after = idx;
        let t0 = self.timestamps[before];
        let t1 = self.timestamps[after];
        let alpha = (timestamp - t0) / (t1 - t0);
        Some(interpolate(self.values[before], self.values[after], alpha))
    }

    pub fn latest(&self) -> Option<GenericTransform> {
        self.values.last().copied()
    }

    pub fn time_range(&self) -> Option<(f64, f64)> {
        Some((*self.timestamps.first()?, *self.timestamps.last()?))
    }

    pub fn in_range(&self, timestamp: f64) -> bool {
        self.time_range()
            .map(|(start, end)| timestamp >= start && timestamp <= end)
            .unwrap_or(false)
    }

    pub fn size(&self) -> usize {
        self.timestamps.len()
    }

    pub fn empty(&self) -> bool {
        self.timestamps.is_empty()
    }

    pub fn clear(&mut self) {
        self.timestamps.clear();
        self.values.clear();
    }

    fn sort_by_time(&mut self) {
        let mut pairs: Vec<_> = self
            .timestamps
            .iter()
            .copied()
            .zip(self.values.iter().copied())
            .collect();
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        self.timestamps = pairs.iter().map(|(t, _)| *t).collect();
        self.values = pairs.into_iter().map(|(_, v)| v).collect();
    }
}

type EdgeKey = (u32, u32);
type PathCacheKey = (u32, u32);

#[derive(Debug, Clone, Default)]
pub struct TimedTransformTree {
    graph: FrameGraph,
    dynamic_transforms: HashMap<EdgeKey, TimedTransformBuffer<100>>,
    static_transforms: HashMap<EdgeKey, GenericTransform>,
    static_edges: HashSet<EdgeKey>,
    edge_directions: HashMap<EdgeKey, (u32, u32)>,
    path_cache: HashMap<PathCacheKey, Vec<VertexId<FrameInfo>>>,
}

impl TimedTransformTree {
    pub fn new() -> Self {
        Self::default()
    }

    fn edge_key(a: VertexId<FrameInfo>, b: VertexId<FrameInfo>) -> EdgeKey {
        let a = a.value();
        let b = b.value();
        if a < b { (a, b) } else { (b, a) }
    }

    fn path_key(a: VertexId<FrameInfo>, b: VertexId<FrameInfo>) -> PathCacheKey {
        (a.value(), b.value())
    }

    pub fn register_frame<Frame: 'static>(
        &mut self,
        name: impl Into<String>,
    ) -> VertexId<FrameInfo> {
        self.path_cache.clear();
        self.graph.register_frame::<Frame>(name)
    }

    pub fn set_transform<To, From, T>(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        tf: Transform<To, From, T>,
        timestamp: f64,
    ) {
        let Some(from_id) = self.graph.get_frame_id(from_frame) else {
            return;
        };
        let Some(to_id) = self.graph.get_frame_id(to_frame) else {
            return;
        };

        let key = Self::edge_key(from_id, to_id);
        self.static_edges.remove(&key);
        self.static_transforms.remove(&key);
        self.edge_directions
            .insert(key, (from_id.value(), to_id.value()));
        self.dynamic_transforms.entry(key).or_default().add(
            timestamp,
            GenericTransform::new(tf.rotation.quat, tf.translation),
        );
        self.graph.ensure_edge(from_id, to_id);
        self.path_cache.clear();
    }

    pub fn set_static_transform<To, From, T>(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        tf: Transform<To, From, T>,
    ) {
        let Some(from_id) = self.graph.get_frame_id(from_frame) else {
            return;
        };
        let Some(to_id) = self.graph.get_frame_id(to_frame) else {
            return;
        };

        let key = Self::edge_key(from_id, to_id);
        self.dynamic_transforms.remove(&key);
        self.static_edges.insert(key);
        self.edge_directions
            .insert(key, (from_id.value(), to_id.value()));
        self.static_transforms
            .insert(key, GenericTransform::new(tf.rotation.quat, tf.translation));
        self.graph.ensure_edge(from_id, to_id);
        self.path_cache.clear();
    }

    pub fn set_static<To, From, T>(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        tf: Transform<To, From, T>,
    ) {
        self.set_static_transform(to_frame, from_frame, tf);
    }

    pub fn lookup(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        timestamp: f64,
    ) -> Option<GenericTransform> {
        if to_frame == from_frame {
            return self
                .graph
                .has_frame(to_frame)
                .then_some(GenericTransform::identity());
        }
        let from_id = self.graph.get_frame_id(from_frame)?;
        let to_id = self.graph.get_frame_id(to_frame)?;
        let path = self.find_path(from_id, to_id);
        if path.is_empty() {
            return None;
        }
        self.compose_path_at_time(&path, timestamp)
    }

    pub fn lookup_latest(&mut self, to_frame: &str, from_frame: &str) -> Option<GenericTransform> {
        if to_frame == from_frame {
            return self
                .graph
                .has_frame(to_frame)
                .then_some(GenericTransform::identity());
        }
        let from_id = self.graph.get_frame_id(from_frame)?;
        let to_id = self.graph.get_frame_id(to_frame)?;
        let path = self.find_path(from_id, to_id);
        if path.is_empty() {
            return None;
        }
        let timestamp = self.find_common_latest_time(&path)?;
        self.compose_path_at_time(&path, timestamp)
    }

    pub fn has_transform(&self, to_frame: &str, from_frame: &str) -> bool {
        let Some(from_id) = self.graph.get_frame_id(from_frame) else {
            return false;
        };
        let Some(to_id) = self.graph.get_frame_id(to_frame) else {
            return false;
        };
        let key = Self::edge_key(from_id, to_id);
        self.static_edges.contains(&key) || self.dynamic_transforms.contains_key(&key)
    }

    pub fn can_transform(&mut self, to_frame: &str, from_frame: &str, timestamp: f64) -> bool {
        if to_frame == from_frame {
            return self.graph.has_frame(to_frame);
        }
        let Some(from_id) = self.graph.get_frame_id(from_frame) else {
            return false;
        };
        let Some(to_id) = self.graph.get_frame_id(to_frame) else {
            return false;
        };
        let path = self.find_path(from_id, to_id);
        if path.is_empty() {
            return false;
        }
        path.windows(2).all(|edge| {
            let key = Self::edge_key(edge[0], edge[1]);
            if self.static_edges.contains(&key) {
                return true;
            }
            self.dynamic_transforms
                .get(&key)
                .map(|buf| buf.in_range(timestamp))
                .unwrap_or(false)
        })
    }

    pub fn time_range(&mut self, to_frame: &str, from_frame: &str) -> Option<(f64, f64)> {
        if to_frame == from_frame {
            return self
                .graph
                .has_frame(to_frame)
                .then_some((-f64::MAX, f64::MAX));
        }
        let from_id = self.graph.get_frame_id(from_frame)?;
        let to_id = self.graph.get_frame_id(to_frame)?;
        let path = self.find_path(from_id, to_id);
        if path.is_empty() {
            return None;
        }

        let mut start = -f64::MAX;
        let mut end = f64::MAX;
        for edge in path.windows(2) {
            let key = Self::edge_key(edge[0], edge[1]);
            if self.static_edges.contains(&key) {
                continue;
            }
            let (edge_start, edge_end) = self.dynamic_transforms.get(&key)?.time_range()?;
            start = start.max(edge_start);
            end = end.min(edge_end);
        }
        (start <= end).then_some((start, end))
    }

    pub fn has_frame(&self, name: &str) -> bool {
        self.graph.has_frame(name)
    }

    pub fn frame_names(&self) -> Vec<String> {
        self.graph.frame_names()
    }

    pub fn frame_count(&self) -> usize {
        self.graph.frame_count()
    }

    pub fn graph(&self) -> &FrameGraph {
        &self.graph
    }

    pub fn clear(&mut self) {
        self.graph.clear();
        self.dynamic_transforms.clear();
        self.static_transforms.clear();
        self.static_edges.clear();
        self.edge_directions.clear();
        self.path_cache.clear();
    }

    fn find_path(
        &mut self,
        from: VertexId<FrameInfo>,
        to: VertexId<FrameInfo>,
    ) -> Vec<VertexId<FrameInfo>> {
        if from == to {
            return vec![from];
        }
        let key = Self::path_key(from, to);
        if let Some(path) = self.path_cache.get(&key) {
            return path.clone();
        }
        let path = self.graph.find_path(from, to);
        self.path_cache.insert(key, path.clone());
        path
    }

    fn get_edge_transform_at_time(
        &self,
        from: VertexId<FrameInfo>,
        to: VertexId<FrameInfo>,
        timestamp: f64,
    ) -> Option<GenericTransform> {
        let key = Self::edge_key(from, to);
        if self.static_edges.contains(&key) {
            return self
                .static_transforms
                .get(&key)
                .map(|tf| self.get_directed_transform(key, *tf, from, to));
        }
        self.dynamic_transforms
            .get(&key)
            .and_then(|buf| buf.lookup(timestamp))
            .map(|tf| self.get_directed_transform(key, tf, from, to))
    }

    fn get_directed_transform(
        &self,
        key: EdgeKey,
        stored_tf: GenericTransform,
        from: VertexId<FrameInfo>,
        to: VertexId<FrameInfo>,
    ) -> GenericTransform {
        match self.edge_directions.get(&key) {
            Some(&(stored_from, stored_to))
                if stored_from == from.value() && stored_to == to.value() =>
            {
                stored_tf
            }
            Some(_) => stored_tf.inverse(),
            None => stored_tf,
        }
    }

    fn find_common_latest_time(&self, path: &[VertexId<FrameInfo>]) -> Option<f64> {
        if path.len() <= 1 {
            return Some(0.0);
        }
        let mut min_latest = f64::MAX;
        let mut has_dynamic = false;
        for edge in path.windows(2) {
            let key = Self::edge_key(edge[0], edge[1]);
            if self.static_edges.contains(&key) {
                continue;
            }
            let (_, latest) = self.dynamic_transforms.get(&key)?.time_range()?;
            min_latest = min_latest.min(latest);
            has_dynamic = true;
        }
        if has_dynamic {
            Some(min_latest)
        } else {
            Some(0.0)
        }
    }

    fn compose_path_at_time(
        &self,
        path: &[VertexId<FrameInfo>],
        timestamp: f64,
    ) -> Option<GenericTransform> {
        if path.is_empty() {
            return None;
        }
        if path.len() == 1 {
            return Some(GenericTransform::identity());
        }
        let mut result = GenericTransform::identity();
        for edge in path.windows(2) {
            let tf = self.get_edge_transform_at_time(edge[0], edge[1], timestamp)?;
            result = tf * result;
        }
        Some(result)
    }
}
