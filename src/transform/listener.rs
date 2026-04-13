use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use crate::{
    frame::Transform,
    transform::{FrameGraph, GenericTransform, TimedTransformTree, TransformTree},
};

pub type TransformCallback = Arc<dyn Fn(&GenericTransform) + Send + Sync + 'static>;
pub type AnyTransformCallback = Arc<dyn Fn(&str, &str, &GenericTransform) + Send + Sync + 'static>;

type StringEdgeKey = (String, String);

#[derive(Default)]
struct ListenerState {
    edge_listeners: HashMap<StringEdgeKey, Vec<(usize, TransformCallback)>>,
    any_listeners: Vec<(usize, AnyTransformCallback)>,
    listener_to_edge: HashMap<usize, StringEdgeKey>,
    next_listener_id: usize,
}

impl ListenerState {
    fn new() -> Self {
        Self {
            next_listener_id: 1,
            ..Self::default()
        }
    }
}

#[derive(Default)]
pub struct TransformListenerMixin {
    state: Mutex<ListenerState>,
}

impl TransformListenerMixin {
    pub fn new() -> Self {
        Self {
            state: Mutex::new(ListenerState::new()),
        }
    }

    fn make_string_edge_key(a: &str, b: &str) -> StringEdgeKey {
        if a < b {
            (a.to_string(), b.to_string())
        } else {
            (b.to_string(), a.to_string())
        }
    }

    pub fn on_update<F>(&self, to_frame: &str, from_frame: &str, callback: F) -> usize
    where
        F: Fn(&GenericTransform) + Send + Sync + 'static,
    {
        let mut state = self.state.lock().expect("listener mutex poisoned");
        let id = state.next_listener_id;
        state.next_listener_id += 1;
        let key = Self::make_string_edge_key(to_frame, from_frame);
        state
            .edge_listeners
            .entry(key.clone())
            .or_default()
            .push((id, Arc::new(callback)));
        state.listener_to_edge.insert(id, key);
        id
    }

    pub fn on_any_update<F>(&self, callback: F) -> usize
    where
        F: Fn(&str, &str, &GenericTransform) + Send + Sync + 'static,
    {
        let mut state = self.state.lock().expect("listener mutex poisoned");
        let id = state.next_listener_id;
        state.next_listener_id += 1;
        state.any_listeners.push((id, Arc::new(callback)));
        id
    }

    pub fn remove_listener(&self, listener_id: usize) {
        let mut state = self.state.lock().expect("listener mutex poisoned");

        if let Some(key) = state.listener_to_edge.remove(&listener_id) {
            if let Some(listeners) = state.edge_listeners.get_mut(&key) {
                listeners.retain(|(id, _)| *id != listener_id);
                if listeners.is_empty() {
                    state.edge_listeners.remove(&key);
                }
            }
            return;
        }

        state.any_listeners.retain(|(id, _)| *id != listener_id);
    }

    pub fn remove_listeners(&self, to_frame: &str, from_frame: &str) {
        let mut state = self.state.lock().expect("listener mutex poisoned");
        let key = Self::make_string_edge_key(to_frame, from_frame);
        if let Some(listeners) = state.edge_listeners.remove(&key) {
            for (id, _) in listeners {
                state.listener_to_edge.remove(&id);
            }
        }
    }

    pub fn clear_listeners(&self) {
        let mut state = self.state.lock().expect("listener mutex poisoned");
        state.edge_listeners.clear();
        state.any_listeners.clear();
        state.listener_to_edge.clear();
    }

    pub fn edge_listener_count(&self) -> usize {
        let state = self.state.lock().expect("listener mutex poisoned");
        state.edge_listeners.values().map(Vec::len).sum()
    }

    pub fn any_listener_count(&self) -> usize {
        let state = self.state.lock().expect("listener mutex poisoned");
        state.any_listeners.len()
    }

    pub fn notify_listeners(&self, to_frame: &str, from_frame: &str, tf: GenericTransform) {
        let (edge_callbacks, any_callbacks) = {
            let state = self.state.lock().expect("listener mutex poisoned");
            let key = Self::make_string_edge_key(to_frame, from_frame);
            let edge_callbacks = state
                .edge_listeners
                .get(&key)
                .map(|listeners| {
                    listeners
                        .iter()
                        .map(|(_, callback)| Arc::clone(callback))
                        .collect::<Vec<_>>()
                })
                .unwrap_or_default();
            let any_callbacks = state
                .any_listeners
                .iter()
                .map(|(_, callback)| Arc::clone(callback))
                .collect::<Vec<_>>();
            (edge_callbacks, any_callbacks)
        };

        for callback in edge_callbacks {
            callback(&tf);
        }
        for callback in any_callbacks {
            callback(to_frame, from_frame, &tf);
        }
    }
}

pub struct ListenableTransformTree {
    listeners: TransformListenerMixin,
    tree: TransformTree,
}

impl Default for ListenableTransformTree {
    fn default() -> Self {
        Self {
            listeners: TransformListenerMixin::new(),
            tree: TransformTree::new(),
        }
    }
}

impl ListenableTransformTree {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn on_update<F>(&self, to_frame: &str, from_frame: &str, callback: F) -> usize
    where
        F: Fn(&GenericTransform) + Send + Sync + 'static,
    {
        self.listeners.on_update(to_frame, from_frame, callback)
    }

    pub fn on_any_update<F>(&self, callback: F) -> usize
    where
        F: Fn(&str, &str, &GenericTransform) + Send + Sync + 'static,
    {
        self.listeners.on_any_update(callback)
    }

    pub fn remove_listener(&self, listener_id: usize) {
        self.listeners.remove_listener(listener_id);
    }

    pub fn remove_listeners(&self, to_frame: &str, from_frame: &str) {
        self.listeners.remove_listeners(to_frame, from_frame);
    }

    pub fn clear_listeners(&self) {
        self.listeners.clear_listeners();
    }

    pub fn edge_listener_count(&self) -> usize {
        self.listeners.edge_listener_count()
    }

    pub fn any_listener_count(&self) -> usize {
        self.listeners.any_listener_count()
    }

    pub fn register_frame<Frame: 'static>(&mut self, name: impl Into<String>) {
        self.tree.register_frame::<Frame>(name);
    }

    pub fn set_transform<To, From, T>(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        tf: Transform<To, From, T>,
    ) {
        let generic = GenericTransform::new(tf.rotation.quat, tf.translation);
        self.tree.set_transform(to_frame, from_frame, tf);
        self.listeners
            .notify_listeners(to_frame, from_frame, generic);
    }

    pub fn lookup(&mut self, to_frame: &str, from_frame: &str) -> Option<GenericTransform> {
        self.tree.lookup(to_frame, from_frame)
    }

    pub fn can_transform(&mut self, to_frame: &str, from_frame: &str) -> bool {
        self.tree.can_transform(to_frame, from_frame)
    }

    pub fn get_path(&mut self, from_frame: &str, to_frame: &str) -> Option<Vec<String>> {
        self.tree.get_path(from_frame, to_frame)
    }

    pub fn has_frame(&self, name: &str) -> bool {
        self.tree.has_frame(name)
    }

    pub fn frame_names(&self) -> Vec<String> {
        self.tree.frame_names()
    }

    pub fn frame_count(&self) -> usize {
        self.tree.frame_count()
    }

    pub fn transform_count(&self) -> usize {
        self.tree.transform_count()
    }

    pub fn clear(&mut self) {
        self.tree.clear();
        self.listeners.clear_listeners();
    }

    pub fn graph(&self) -> &FrameGraph {
        self.tree.graph()
    }
}

pub struct ListenableTimedTransformTree {
    listeners: TransformListenerMixin,
    tree: TimedTransformTree,
}

impl Default for ListenableTimedTransformTree {
    fn default() -> Self {
        Self {
            listeners: TransformListenerMixin::new(),
            tree: TimedTransformTree::new(),
        }
    }
}

impl ListenableTimedTransformTree {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn on_update<F>(&self, to_frame: &str, from_frame: &str, callback: F) -> usize
    where
        F: Fn(&GenericTransform) + Send + Sync + 'static,
    {
        self.listeners.on_update(to_frame, from_frame, callback)
    }

    pub fn on_any_update<F>(&self, callback: F) -> usize
    where
        F: Fn(&str, &str, &GenericTransform) + Send + Sync + 'static,
    {
        self.listeners.on_any_update(callback)
    }

    pub fn remove_listener(&self, listener_id: usize) {
        self.listeners.remove_listener(listener_id);
    }

    pub fn remove_listeners(&self, to_frame: &str, from_frame: &str) {
        self.listeners.remove_listeners(to_frame, from_frame);
    }

    pub fn clear_listeners(&self) {
        self.listeners.clear_listeners();
    }

    pub fn edge_listener_count(&self) -> usize {
        self.listeners.edge_listener_count()
    }

    pub fn any_listener_count(&self) -> usize {
        self.listeners.any_listener_count()
    }

    pub fn register_frame<Frame: 'static>(&mut self, name: impl Into<String>) {
        self.tree.register_frame::<Frame>(name);
    }

    pub fn set_transform<To, From, T>(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        tf: Transform<To, From, T>,
        timestamp: f64,
    ) {
        let generic = GenericTransform::new(tf.rotation.quat, tf.translation);
        self.tree.set_transform(to_frame, from_frame, tf, timestamp);
        self.listeners
            .notify_listeners(to_frame, from_frame, generic);
    }

    pub fn set_static_transform<To, From, T>(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        tf: Transform<To, From, T>,
    ) {
        let generic = GenericTransform::new(tf.rotation.quat, tf.translation);
        self.tree.set_static_transform(to_frame, from_frame, tf);
        self.listeners
            .notify_listeners(to_frame, from_frame, generic);
    }

    pub fn lookup(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        timestamp: f64,
    ) -> Option<GenericTransform> {
        self.tree.lookup(to_frame, from_frame, timestamp)
    }

    pub fn lookup_latest(&mut self, to_frame: &str, from_frame: &str) -> Option<GenericTransform> {
        self.tree.lookup_latest(to_frame, from_frame)
    }

    pub fn has_transform(&self, to_frame: &str, from_frame: &str) -> bool {
        self.tree.has_transform(to_frame, from_frame)
    }

    pub fn can_transform(&mut self, to_frame: &str, from_frame: &str, timestamp: f64) -> bool {
        self.tree.can_transform(to_frame, from_frame, timestamp)
    }

    pub fn time_range(&mut self, to_frame: &str, from_frame: &str) -> Option<(f64, f64)> {
        self.tree.time_range(to_frame, from_frame)
    }

    pub fn has_frame(&self, name: &str) -> bool {
        self.tree.has_frame(name)
    }

    pub fn frame_names(&self) -> Vec<String> {
        self.tree.frame_names()
    }

    pub fn frame_count(&self) -> usize {
        self.tree.frame_count()
    }

    pub fn clear(&mut self) {
        self.tree.clear();
        self.listeners.clear_listeners();
    }

    pub fn graph(&self) -> &FrameGraph {
        // temporal tree doesn't expose graph directly currently; use inner graph through method if added later
        self.tree.graph()
    }
}
