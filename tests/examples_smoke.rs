use datapod::{Point, Quaternion};

use concord::{
    Geo, TimedTransformTree, Transform, TransformTree, Wgs, convert,
    frame::{Enu, Ned},
};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Base;
#[derive(Debug, Clone, Copy)]
struct Camera;
#[derive(Debug, Clone, Copy)]
struct Odom;

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

#[test]
fn example_wgs_to_enu_flow_works() {
    let origin = Geo::new(52.0, 4.0, 10.0);
    let point = Wgs::new(52.0001, 4.0002, 12.0);

    let enu = convert(point)
        .with_ref(origin)
        .to::<Enu>()
        .build()
        .expect("valid ENU conversion");

    assert_eq!(enu.origin, origin);
    assert!(enu.distance_from_origin() > 0.0);
}

#[test]
fn example_builder_chain_roundtrips() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let point = Wgs::new(48.8570, 2.3530, 40.0);

    let roundtrip = convert(point)
        .with_ref(origin)
        .to::<Enu>()
        .to::<Ned>()
        .to::<Wgs>()
        .build()
        .expect("roundtrip");

    approx_eq(roundtrip.latitude, point.latitude, 1e-10);
    approx_eq(roundtrip.longitude, point.longitude, 1e-10);
    approx_eq(roundtrip.altitude, point.altitude, 1e-5);
}

#[test]
fn example_transform_tree_lookup_works() {
    let mut tree = TransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Base>("base");
    tree.register_frame::<Camera>("camera");

    tree.set_transform(
        "world",
        "base",
        Transform::<World, Base>::from_qt(Quaternion::identity(), Point::new(1.0, 2.0, 0.0)),
    );
    tree.set_transform(
        "base",
        "camera",
        Transform::<Base, Camera>::from_qt(Quaternion::identity(), Point::new(0.0, 0.0, 1.0)),
    );

    let tf = tree.lookup("world", "camera").expect("path exists");
    assert_eq!(tf.translation, Point::new(1.0, 2.0, 1.0));
}

#[test]
fn example_timed_transform_lookup_works() {
    let mut tree = TimedTransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");

    tree.set_transform(
        "world",
        "odom",
        Transform::<World, Odom>::from_qt(Quaternion::identity(), Point::new(0.0, 0.0, 0.0)),
        1.0,
    );
    tree.set_transform(
        "world",
        "odom",
        Transform::<World, Odom>::from_qt(Quaternion::identity(), Point::new(10.0, 0.0, 0.0)),
        2.0,
    );

    let tf = tree
        .lookup("world", "odom", 1.5)
        .expect("interpolated transform");
    assert_eq!(tf.translation, Point::new(5.0, 0.0, 0.0));
}
