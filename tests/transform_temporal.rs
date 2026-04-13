use glam::{DQuat, DVec3};

use concord::{
    GenericTransform, TimedTransformBuffer, TimedTransformTree, Transform, interpolate, lerp, slerp,
};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Odom;
#[derive(Debug, Clone, Copy)]
struct BaseLink;
#[derive(Debug, Clone, Copy)]
struct Camera;

fn make_translation<To, From>(x: f64, y: f64, z: f64) -> Transform<To, From> {
    Transform::from_qt(DQuat::IDENTITY, DVec3::new(x, y, z))
}

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

fn approx_vec(a: DVec3, b: DVec3, eps: f64) {
    assert!((a - b).length() < eps, "left={a:?}, right={b:?}");
}

fn approx_quat(a: DQuat, b: DQuat, eps: f64) {
    let da = [a.x, a.y, a.z, a.w];
    let db = [b.x, b.y, b.z, b.w];
    let same = da.iter().zip(db.iter()).all(|(x, y)| (*x - *y).abs() < eps);
    let opposite = da.iter().zip(db.iter()).all(|(x, y)| (*x + *y).abs() < eps);
    assert!(same || opposite, "left={a:?}, right={b:?}");
}

#[test]
fn interpolation_helpers_work() {
    let q1 = DQuat::IDENTITY;
    let q2 = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    approx_quat(slerp(q1, q1, 0.5), q1, 1e-10);
    approx_quat(slerp(q1, q2, 0.0), q1, 1e-10);
    approx_quat(slerp(q1, q2, 1.0), q2, 1e-10);

    approx_vec(
        lerp(DVec3::ZERO, DVec3::new(10.0, 20.0, 30.0), 0.25),
        DVec3::new(2.5, 5.0, 7.5),
        1e-12,
    );

    let tf1 = GenericTransform::new(DQuat::IDENTITY, DVec3::ZERO);
    let tf2 = GenericTransform::new(q2, DVec3::new(10.0, 0.0, 0.0));
    let mid = interpolate(tf1, tf2, 0.5);
    approx_eq(mid.translation.x, 5.0, 1e-12);
}

#[test]
fn timed_transform_buffer_basic_and_interpolation_behavior() {
    let mut buffer = TimedTransformBuffer::<5>::new();
    assert!(buffer.empty());
    assert_eq!(buffer.size(), 0);
    assert!(buffer.latest().is_none());
    assert!(buffer.time_range().is_none());

    let tf1 = GenericTransform::new(DQuat::IDENTITY, DVec3::new(0.0, 0.0, 0.0));
    let tf2 = GenericTransform::new(DQuat::IDENTITY, DVec3::new(10.0, 0.0, 0.0));
    buffer.add(1.0, tf1);
    buffer.add(2.0, tf2);

    assert_eq!(buffer.size(), 2);
    assert!(buffer.in_range(1.5));
    assert!(!buffer.in_range(3.0));
    approx_eq(buffer.latest().unwrap().translation.x, 10.0, 1e-12);
    assert_eq!(buffer.time_range(), Some((1.0, 2.0)));

    approx_eq(buffer.lookup(1.0).unwrap().translation.x, 0.0, 1e-12);
    approx_eq(buffer.lookup(1.5).unwrap().translation.x, 5.0, 1e-12);
    assert!(buffer.lookup(0.5).is_none());
    assert!(buffer.lookup(3.0).is_none());

    let mut overflow = TimedTransformBuffer::<3>::new();
    for i in 0..5 {
        overflow.add(i as f64, tf1);
    }
    assert!(overflow.size() <= 3);
    let range = overflow.time_range().unwrap();
    assert!(range.0 >= 2.0);
    assert_eq!(range.1, 4.0);
}

#[test]
fn timed_transform_tree_static_dynamic_and_interpolated_lookup() {
    let mut tree = TimedTransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");
    tree.register_frame::<BaseLink>("base_link");
    tree.register_frame::<Camera>("camera");

    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(0.0, 0.0, 0.0),
        1.0,
    );
    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(10.0, 0.0, 0.0),
        2.0,
    );
    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 5.0, 0.0),
        1.0,
    );
    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 5.0, 0.0),
        2.0,
    );
    tree.set_static_transform(
        "base_link",
        "camera",
        make_translation::<BaseLink, Camera>(0.0, 0.0, 1.0),
    );

    assert!(tree.has_transform("world", "odom"));
    assert!(tree.can_transform("world", "camera", 1.5));
    assert!(!tree.can_transform("world", "camera", 0.5));

    let result = tree
        .lookup("world", "camera", 1.5)
        .expect("temporal lookup");
    approx_vec(result.translation, DVec3::new(5.0, 5.0, 1.0), 1e-12);

    let exact = tree.lookup("world", "odom", 1.0).expect("exact lookup");
    approx_eq(exact.translation.x, 0.0, 1e-12);
}

#[test]
fn timed_transform_tree_time_range_and_lookup_latest() {
    let mut tree = TimedTransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");
    tree.register_frame::<BaseLink>("base_link");

    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 0.0, 0.0),
        1.0,
    );
    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(5.0, 0.0, 0.0),
        5.0,
    );
    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 1.0, 0.0),
        1.0,
    );
    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 3.0, 0.0),
        3.0,
    );

    assert_eq!(tree.time_range("world", "odom"), Some((1.0, 5.0)));
    assert_eq!(tree.time_range("world", "base_link"), Some((1.0, 3.0)));

    let latest = tree.lookup_latest("world", "base_link").expect("latest");
    approx_vec(latest.translation, DVec3::new(3.0, 3.0, 0.0), 1e-12);

    tree.clear();
    assert_eq!(tree.frame_count(), 0);
    assert!(tree.frame_names().is_empty());
}
