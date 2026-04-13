use std::collections::HashMap;

use graphix::vertex::VertexId;

use crate::{
    frame::Transform,
    transform::{FrameGraph, FrameInfo, GenericTransform},
};

type PathCacheKey = (u32, u32);

#[derive(Debug, Clone, Default)]
pub struct TransformTree {
    graph: FrameGraph,
    path_cache: HashMap<PathCacheKey, Vec<VertexId<FrameInfo>>>,
}

impl TransformTree {
    pub fn new() -> Self {
        Self::default()
    }

    fn cache_key(from: VertexId<FrameInfo>, to: VertexId<FrameInfo>) -> PathCacheKey {
        (from.value(), to.value())
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
    ) {
        self.path_cache.clear();
        self.graph.set_transform(to_frame, from_frame, tf);
    }

    pub fn lookup(&mut self, to_frame: &str, from_frame: &str) -> Option<GenericTransform> {
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
        self.compose_path(&path)
    }

    pub fn can_transform(&mut self, to_frame: &str, from_frame: &str) -> bool {
        if to_frame == from_frame {
            return self.graph.has_frame(to_frame);
        }

        let Some(from_id) = self.graph.get_frame_id(from_frame) else {
            return false;
        };
        let Some(to_id) = self.graph.get_frame_id(to_frame) else {
            return false;
        };

        !self.find_path(from_id, to_id).is_empty()
    }

    pub fn get_path(&mut self, from_frame: &str, to_frame: &str) -> Option<Vec<String>> {
        if from_frame == to_frame {
            return self
                .graph
                .has_frame(from_frame)
                .then_some(vec![from_frame.to_string()]);
        }

        let from_id = self.graph.get_frame_id(from_frame)?;
        let to_id = self.graph.get_frame_id(to_frame)?;
        let path = self.find_path(from_id, to_id);
        if path.is_empty() {
            return None;
        }

        Some(
            path.into_iter()
                .map(|v| self.graph.get_frame_name(v))
                .collect(),
        )
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

    pub fn transform_count(&self) -> usize {
        self.graph.transform_count()
    }

    pub fn clear(&mut self) {
        self.graph.clear();
        self.path_cache.clear();
    }

    pub fn graph(&self) -> &FrameGraph {
        &self.graph
    }

    fn find_path(
        &mut self,
        from: VertexId<FrameInfo>,
        to: VertexId<FrameInfo>,
    ) -> Vec<VertexId<FrameInfo>> {
        if from == to {
            return vec![from];
        }

        let key = Self::cache_key(from, to);
        if let Some(path) = self.path_cache.get(&key) {
            return path.clone();
        }

        let path = self.graph.find_path(from, to);
        self.path_cache.insert(key, path.clone());
        path
    }

    fn compose_path(&self, path: &[VertexId<FrameInfo>]) -> Option<GenericTransform> {
        if path.is_empty() {
            return None;
        }
        if path.len() == 1 {
            return Some(GenericTransform::identity());
        }

        let mut result = GenericTransform::identity();
        for edge in path.windows(2) {
            let current = edge[0];
            let next = edge[1];
            let tf = self.graph.get_generic_transform(current, next)?;
            result = tf * result;
        }
        Some(result)
    }
}
