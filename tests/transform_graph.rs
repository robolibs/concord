use datapod::{Point, Quaternion};

use concord::{FrameGraph, GenericTransform, Transform};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Odom;
#[derive(Debug, Clone, Copy)]
struct BaseLink;
#[derive(Debug, Clone, Copy)]
struct Camera;

fn make_translation<To, From>(x: f64, y: f64, z: f64) -> Transform<To, From> {
    Transform::from_qt(Quaternion::identity(), Point::new(x, y, z))
}

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

#[test]
fn frame_graph_basic_registration() {
    let mut graph = FrameGraph::new();
    assert_eq!(graph.frame_count(), 0);
    assert_eq!(graph.transform_count(), 0);
    assert!(graph.frame_names().is_empty());

    let world_id = graph.register_frame::<World>("world");
    assert_eq!(graph.frame_count(), 1);
    assert!(graph.has_frame("world"));
    assert!(!graph.has_frame("odom"));
    assert_eq!(graph.get_frame_id("world"), Some(world_id));

    graph.register_frame::<Odom>("odom");
    graph.register_frame::<BaseLink>("base_link");
    let names = graph.frame_names();
    assert_eq!(
        names,
        vec![
            "base_link".to_string(),
            "odom".to_string(),
            "world".to_string()
        ]
    );

    graph.register_frame::<Camera>("world");
    assert_eq!(graph.frame_count(), 3);
}

#[test]
fn frame_graph_set_get_and_update_transform() {
    let mut graph = FrameGraph::new();
    graph.register_frame::<World>("world");
    graph.register_frame::<Odom>("odom");
    graph.register_frame::<BaseLink>("base_link");

    let tf = make_translation::<World, Odom>(1.0, 2.0, 3.0);
    graph.set_transform("world", "odom", tf);

    assert!(graph.has_transform("world", "odom"));
    assert_eq!(graph.transform_count(), 1);

    let result = graph
        .get_transform("world", "odom")
        .expect("direct transform");
    approx_eq(result.translation.x, 1.0, 1e-12);
    approx_eq(result.translation.y, 2.0, 1e-12);
    approx_eq(result.translation.z, 3.0, 1e-12);

    let inverse = graph
        .get_transform("odom", "world")
        .expect("inverse transform");
    approx_eq(inverse.translation.x, -1.0, 1e-12);
    approx_eq(inverse.translation.y, -2.0, 1e-12);
    approx_eq(inverse.translation.z, -3.0, 1e-12);

    let tf2 = make_translation::<World, Odom>(2.0, 0.0, 0.0);
    graph.set_transform("world", "odom", tf2);
    assert_eq!(graph.transform_count(), 1);
    let updated = graph
        .get_transform("world", "odom")
        .expect("updated transform");
    approx_eq(updated.translation.x, 2.0, 1e-12);
}

#[test]
fn generic_transform_supports_identity_inverse_apply_and_compose() {
    let identity = GenericTransform::identity();
    let point = Point::new(1.0, 2.0, 3.0);
    assert_eq!(identity.apply(point), point);

    let tf = GenericTransform::new(Quaternion::identity(), Point::new(5.0, 6.0, 7.0));
    assert_eq!(
        tf.apply(Point::new(0.0, 0.0, 0.0)),
        Point::new(5.0, 6.0, 7.0)
    );
    assert_eq!(tf.inverse().apply(tf.apply(point)), point);

    let a = GenericTransform::new(Quaternion::identity(), Point::new(1.0, 0.0, 0.0));
    let b = GenericTransform::new(Quaternion::identity(), Point::new(0.0, 2.0, 0.0));
    let c = a * b;
    assert_eq!(c.translation, Point::new(1.0, 2.0, 0.0));
}

#[test]
fn frame_graph_path_existence_basics() {
    let mut graph = FrameGraph::new();
    graph.register_frame::<World>("world");
    graph.register_frame::<Odom>("odom");
    graph.register_frame::<BaseLink>("base_link");
    graph.register_frame::<Camera>("camera");

    graph.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 0.0, 0.0),
    );
    graph.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 1.0, 0.0),
    );
    graph.set_transform(
        "base_link",
        "camera",
        make_translation::<BaseLink, Camera>(0.0, 0.0, 1.0),
    );

    assert!(graph.has_path("world", "odom"));
    assert!(graph.has_path("world", "base_link"));
    assert!(graph.has_path("world", "camera"));
    assert!(graph.has_path("camera", "world"));
    assert!(!graph.has_path("world", "missing"));

    let world = graph.get_frame_id("world").unwrap();
    let camera = graph.get_frame_id("camera").unwrap();
    let path = graph.find_path(camera, world);
    assert_eq!(path.len(), 4);
}
