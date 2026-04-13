use std::any::TypeId;
use std::collections::HashMap;
use std::ops::Mul;

use glam::{DQuat, DVec3};
use graphix::vertex::algorithms::{bfs_to, reconstruct_bfs_path};
use graphix::vertex::{EdgeType, Graph, VertexId};

use crate::frame::Transform;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GenericTransform {
    pub rotation: DQuat,
    pub translation: DVec3,
}

impl Default for GenericTransform {
    fn default() -> Self {
        Self::identity()
    }
}

impl GenericTransform {
    pub fn new(rotation: DQuat, translation: DVec3) -> Self {
        Self {
            rotation: rotation.normalize(),
            translation,
        }
    }

    pub fn identity() -> Self {
        Self::new(DQuat::IDENTITY, DVec3::ZERO)
    }

    pub fn inverse(self) -> Self {
        let rotation = self.rotation.conjugate();
        let translation = -(rotation * self.translation);
        Self::new(rotation, translation)
    }

    pub fn apply(self, point: DVec3) -> DVec3 {
        self.rotation * point + self.translation
    }
}

impl Mul<GenericTransform> for GenericTransform {
    type Output = GenericTransform;

    fn mul(self, rhs: GenericTransform) -> Self::Output {
        GenericTransform::new(
            self.rotation * rhs.rotation,
            self.rotation * rhs.translation + self.translation,
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FrameInfo {
    pub name: String,
    pub type_id: TypeId,
}

impl Default for FrameInfo {
    fn default() -> Self {
        Self {
            name: String::new(),
            type_id: TypeId::of::<()>(),
        }
    }
}

type EdgeKey = (u32, u32);

#[derive(Debug, Clone)]
struct InternalTransformData {
    generic: GenericTransform,
    from_id: u32,
    to_id: u32,
}

#[derive(Debug, Clone, Default)]
pub struct FrameGraph {
    graph: Graph<FrameInfo, ()>,
    name_to_vertex: HashMap<String, VertexId<FrameInfo>>,
    vertex_to_name: HashMap<u32, String>,
    type_to_vertex: HashMap<TypeId, VertexId<FrameInfo>>,
    transforms: HashMap<EdgeKey, InternalTransformData>,
}

impl FrameGraph {
    pub fn new() -> Self {
        Self::default()
    }

    fn make_edge_key(a: VertexId<FrameInfo>, b: VertexId<FrameInfo>) -> EdgeKey {
        let a = a.value();
        let b = b.value();
        if a < b { (a, b) } else { (b, a) }
    }

    pub fn register_frame<Frame: 'static>(
        &mut self,
        name: impl Into<String>,
    ) -> VertexId<FrameInfo> {
        let name = name.into();
        if let Some(id) = self.name_to_vertex.get(&name).copied() {
            return id;
        }

        let info = FrameInfo {
            name: name.clone(),
            type_id: TypeId::of::<Frame>(),
        };
        let vertex_id = self.graph.add_vertex(info);
        self.name_to_vertex.insert(name.clone(), vertex_id);
        self.vertex_to_name.insert(vertex_id.value(), name);
        self.type_to_vertex.insert(TypeId::of::<Frame>(), vertex_id);
        vertex_id
    }

    pub fn has_frame(&self, name: &str) -> bool {
        self.name_to_vertex.contains_key(name)
    }

    pub fn has_frame_type<Frame: 'static>(&self) -> bool {
        self.type_to_vertex.contains_key(&TypeId::of::<Frame>())
    }

    pub fn get_frame_id(&self, name: &str) -> Option<VertexId<FrameInfo>> {
        self.name_to_vertex.get(name).copied()
    }

    pub fn get_frame_id_of<Frame: 'static>(&self) -> Option<VertexId<FrameInfo>> {
        self.type_to_vertex.get(&TypeId::of::<Frame>()).copied()
    }

    pub fn set_transform<To, From, T>(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        tf: Transform<To, From, T>,
    ) {
        let Some(from_id) = self.get_frame_id(from_frame) else {
            return;
        };
        let Some(to_id) = self.get_frame_id(to_frame) else {
            return;
        };

        let edge_key = Self::make_edge_key(from_id, to_id);
        self.transforms.insert(
            edge_key,
            InternalTransformData {
                generic: GenericTransform::new(tf.rotation.quat, tf.translation),
                from_id: from_id.value(),
                to_id: to_id.value(),
            },
        );

        if !self.graph.has_edge(from_id, to_id) {
            self.graph
                .add_edge(from_id, to_id, 1.0, EdgeType::Undirected, ());
        }
    }

    pub fn has_transform(&self, to_frame: &str, from_frame: &str) -> bool {
        let Some(from_id) = self.get_frame_id(from_frame) else {
            return false;
        };
        let Some(to_id) = self.get_frame_id(to_frame) else {
            return false;
        };

        self.transforms
            .contains_key(&Self::make_edge_key(from_id, to_id))
    }

    pub fn get_transform(&self, to_frame: &str, from_frame: &str) -> Option<GenericTransform> {
        let from_id = self.get_frame_id(from_frame)?;
        let to_id = self.get_frame_id(to_frame)?;
        self.get_generic_transform(from_id, to_id)
    }

    pub fn get_generic_transform(
        &self,
        from: VertexId<FrameInfo>,
        to: VertexId<FrameInfo>,
    ) -> Option<GenericTransform> {
        let edge_key = Self::make_edge_key(from, to);
        let data = self.transforms.get(&edge_key)?;
        if data.from_id == from.value() && data.to_id == to.value() {
            Some(data.generic)
        } else {
            Some(data.generic.inverse())
        }
    }

    pub fn frame_names(&self) -> Vec<String> {
        let mut names: Vec<_> = self.vertex_to_name.values().cloned().collect();
        names.sort();
        names
    }

    pub fn frame_count(&self) -> usize {
        self.graph.vertex_count()
    }

    pub fn transform_count(&self) -> usize {
        self.transforms.len()
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }

    pub fn get_frame_info(&self, vertex_id: VertexId<FrameInfo>) -> Option<&FrameInfo> {
        self.graph.get_vertex(vertex_id)
    }

    pub fn get_frame_name(&self, vertex_id: VertexId<FrameInfo>) -> String {
        self.vertex_to_name
            .get(&vertex_id.value())
            .cloned()
            .unwrap_or_default()
    }

    pub fn has_edge(&self, from: VertexId<FrameInfo>, to: VertexId<FrameInfo>) -> bool {
        self.graph.has_edge(from, to)
    }

    pub fn neighbors(&self, vertex: VertexId<FrameInfo>) -> Vec<VertexId<FrameInfo>> {
        self.graph.neighbors(vertex)
    }

    pub fn graph(&self) -> &Graph<FrameInfo, ()> {
        &self.graph
    }

    pub(crate) fn ensure_edge(&mut self, from: VertexId<FrameInfo>, to: VertexId<FrameInfo>) {
        if !self.graph.has_edge(from, to) {
            self.graph.add_edge(from, to, 1.0, EdgeType::Undirected, ());
        }
    }

    pub fn find_path(
        &self,
        from: VertexId<FrameInfo>,
        to: VertexId<FrameInfo>,
    ) -> Vec<VertexId<FrameInfo>> {
        if from == to {
            return vec![from];
        }
        let bfs = bfs_to(&self.graph, from, to);
        if !bfs.target_found && !bfs.distance.contains_key(&to) {
            return Vec::new();
        }
        reconstruct_bfs_path(&bfs, from, to)
    }

    pub fn has_path(&self, to_frame: &str, from_frame: &str) -> bool {
        let Some(from_id) = self.get_frame_id(from_frame) else {
            return false;
        };
        let Some(to_id) = self.get_frame_id(to_frame) else {
            return false;
        };
        !self.find_path(from_id, to_id).is_empty()
    }
}
