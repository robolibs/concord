use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use datapod::{Point, Quaternion};

use concord::{ListenableTimedTransformTree, ListenableTransformTree, Rotation, Transform};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Odom;
#[derive(Debug, Clone, Copy)]
struct BaseLink;

fn make_translation<To, From>(x: f64, y: f64, z: f64) -> Transform<To, From> {
    Transform::from_qt(Quaternion::identity(), Point::new(x, y, z))
}

#[test]
fn listenable_transform_tree_callbacks_register_trigger_and_remove() {
    let mut tree = ListenableTransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");

    let count1 = Arc::new(AtomicUsize::new(0));
    let count2 = Arc::new(AtomicUsize::new(0));
    let last = Arc::new(Mutex::new(Point::new(0.0, 0.0, 0.0)));

    let c1 = Arc::clone(&count1);
    let last_t = Arc::clone(&last);
    let id1 = tree.on_update("world", "odom", move |tf| {
        c1.fetch_add(1, Ordering::SeqCst);
        *last_t.lock().expect("mutex") = tf.translation;
    });

    let c2 = Arc::clone(&count2);
    let id2 = tree.on_update("world", "odom", move |_| {
        c2.fetch_add(1, Ordering::SeqCst);
    });

    assert!(id1 > 0);
    assert!(id2 > 0);
    assert_eq!(tree.edge_listener_count(), 2);

    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 2.0, 3.0),
    );
    assert_eq!(count1.load(Ordering::SeqCst), 1);
    assert_eq!(count2.load(Ordering::SeqCst), 1);
    assert_eq!(*last.lock().expect("mutex"), Point::new(1.0, 2.0, 3.0));

    tree.remove_listener(id1);
    assert_eq!(tree.edge_listener_count(), 1);
    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(2.0, 0.0, 0.0),
    );
    assert_eq!(count1.load(Ordering::SeqCst), 1);
    assert_eq!(count2.load(Ordering::SeqCst), 2);

    tree.remove_listener(id2);
    assert_eq!(tree.edge_listener_count(), 0);
}

#[test]
fn listenable_transform_tree_any_update_and_edge_order_independence() {
    let mut tree = ListenableTransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");
    tree.register_frame::<BaseLink>("base_link");

    let any_count = Arc::new(AtomicUsize::new(0));
    let last_pair = Arc::new(Mutex::new((String::new(), String::new())));
    let any_count_c = Arc::clone(&any_count);
    let last_pair_c = Arc::clone(&last_pair);
    tree.on_any_update(move |to, from, _| {
        any_count_c.fetch_add(1, Ordering::SeqCst);
        *last_pair_c.lock().expect("mutex") = (to.to_string(), from.to_string());
    });
    assert_eq!(tree.any_listener_count(), 1);

    let edge_count = Arc::new(AtomicUsize::new(0));
    let edge_count_c = Arc::clone(&edge_count);
    tree.on_update("odom", "world", move |_| {
        edge_count_c.fetch_add(1, Ordering::SeqCst);
    });

    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 0.0, 0.0),
    );
    assert_eq!(any_count.load(Ordering::SeqCst), 1);
    assert_eq!(edge_count.load(Ordering::SeqCst), 1);
    assert_eq!(
        *last_pair.lock().expect("mutex"),
        ("world".to_string(), "odom".to_string())
    );

    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 1.0, 0.0),
    );
    assert_eq!(any_count.load(Ordering::SeqCst), 2);

    tree.remove_listeners("world", "odom");
    assert_eq!(tree.edge_listener_count(), 0);
}

#[test]
fn listenable_transform_tree_clear_clears_listeners() {
    let mut tree = ListenableTransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");

    let count = Arc::new(AtomicUsize::new(0));
    let count_c = Arc::clone(&count);
    tree.on_update("world", "odom", move |_| {
        count_c.fetch_add(1, Ordering::SeqCst);
    });

    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 0.0, 0.0),
    );
    assert_eq!(count.load(Ordering::SeqCst), 1);

    tree.clear();
    assert_eq!(tree.edge_listener_count(), 0);

    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");
    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(2.0, 0.0, 0.0),
    );
    assert_eq!(count.load(Ordering::SeqCst), 1);
}

#[test]
fn listenable_timed_transform_tree_dynamic_and_static_callbacks_work() {
    let mut tree = ListenableTimedTransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");

    let count = Arc::new(AtomicUsize::new(0));
    let last = Arc::new(Mutex::new(Point::new(0.0, 0.0, 0.0)));
    let count_c = Arc::clone(&count);
    let last_c = Arc::clone(&last);
    tree.on_update("world", "odom", move |tf| {
        count_c.fetch_add(1, Ordering::SeqCst);
        *last_c.lock().expect("mutex") = tf.translation;
    });

    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 2.0, 3.0),
        1.0,
    );
    assert_eq!(count.load(Ordering::SeqCst), 1);
    assert_eq!(*last.lock().expect("mutex"), Point::new(1.0, 2.0, 3.0));

    tree.set_static_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(5.0, 6.0, 7.0),
    );
    assert_eq!(count.load(Ordering::SeqCst), 2);
    assert_eq!(*last.lock().expect("mutex"), Point::new(5.0, 6.0, 7.0));

    let rot = Rotation::<World, Odom>::from_euler_zyx(std::f64::consts::FRAC_PI_4, 0.0, 0.0);
    tree.set_static_transform(
        "world",
        "odom",
        Transform::new(rot, Point::new(1.0, 2.0, 3.0)),
    );
    assert_eq!(count.load(Ordering::SeqCst), 3);
}
