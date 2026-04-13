use glam::{DQuat, DVec3};

use concord::{Rotation, Transform, TransformTree};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Odom;
#[derive(Debug, Clone, Copy)]
struct BaseLink;
#[derive(Debug, Clone, Copy)]
struct Camera;
#[derive(Debug, Clone, Copy)]
struct Lidar;

fn make_translation<To, From>(x: f64, y: f64, z: f64) -> Transform<To, From> {
    Transform::from_qt(DQuat::IDENTITY, DVec3::new(x, y, z))
}

fn make_rotation_z<To, From>(angle_rad: f64) -> Transform<To, From> {
    Transform::new(Rotation::from_euler_zyx(angle_rad, 0.0, 0.0), DVec3::ZERO)
}

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

fn approx_vec(a: DVec3, b: DVec3, eps: f64) {
    assert!((a - b).length() < eps, "left={a:?} right={b:?}");
}

#[test]
fn transform_tree_path_finding_and_can_transform() {
    let mut tree = TransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");
    tree.register_frame::<BaseLink>("base_link");
    tree.register_frame::<Camera>("camera");
    tree.register_frame::<Lidar>("lidar");

    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 0.0, 0.0),
    );
    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 1.0, 0.0),
    );
    tree.set_transform(
        "base_link",
        "camera",
        make_translation::<BaseLink, Camera>(0.5, 0.0, 0.5),
    );
    tree.set_transform(
        "base_link",
        "lidar",
        make_translation::<BaseLink, Lidar>(0.0, 0.0, 1.0),
    );

    assert!(tree.can_transform("world", "odom"));
    assert!(tree.can_transform("odom", "world"));
    assert!(tree.can_transform("world", "camera"));
    assert!(tree.can_transform("camera", "world"));
    assert!(tree.can_transform("lidar", "camera"));
    assert!(!tree.can_transform("world", "missing"));
    assert!(tree.can_transform("world", "world"));
}

#[test]
fn transform_tree_direct_chained_and_inverse_lookup() {
    let mut tree = TransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");
    tree.register_frame::<BaseLink>("base_link");
    tree.register_frame::<Camera>("camera");

    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 0.0, 0.0),
    );
    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 2.0, 0.0),
    );
    tree.set_transform(
        "base_link",
        "camera",
        make_translation::<BaseLink, Camera>(0.0, 0.0, 3.0),
    );

    let direct = tree.lookup("world", "odom").expect("direct");
    approx_vec(direct.translation, DVec3::new(1.0, 0.0, 0.0), 1e-12);

    let chained = tree.lookup("world", "camera").expect("chained");
    approx_vec(chained.translation, DVec3::new(1.0, 2.0, 3.0), 1e-12);
    approx_vec(chained.apply(DVec3::ZERO), DVec3::new(1.0, 2.0, 3.0), 1e-12);

    let inverse = tree.lookup("camera", "world").expect("inverse");
    approx_vec(inverse.translation, DVec3::new(-1.0, -2.0, -3.0), 1e-12);

    let p = DVec3::new(7.0, 11.0, 13.0);
    approx_vec(inverse.apply(chained.apply(p)), p, 1e-10);
}

#[test]
fn transform_tree_rotation_then_translation_composes_correctly() {
    let mut tree = TransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");
    tree.register_frame::<BaseLink>("base_link");

    let angle = std::f64::consts::FRAC_PI_4;
    tree.set_transform("world", "odom", make_rotation_z::<World, Odom>(angle));
    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(1.0, 0.0, 0.0),
    );

    let result = tree.lookup("world", "base_link").expect("composed");
    let expected = DVec3::new(angle.cos(), angle.sin(), 0.0);
    approx_vec(result.apply(DVec3::ZERO), expected, 1e-10);
    approx_vec(result.translation, expected, 1e-10);
}

#[test]
fn transform_tree_get_path_clear_and_edge_cases() {
    let mut tree = TransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");
    tree.register_frame::<BaseLink>("base_link");
    tree.register_frame::<Camera>("camera");
    tree.set_transform(
        "world",
        "odom",
        make_translation::<World, Odom>(1.0, 0.0, 0.0),
    );
    tree.set_transform(
        "odom",
        "base_link",
        make_translation::<Odom, BaseLink>(0.0, 1.0, 0.0),
    );
    tree.set_transform(
        "base_link",
        "camera",
        make_translation::<BaseLink, Camera>(0.0, 0.0, 1.0),
    );

    let path = tree.get_path("world", "camera").expect("path");
    assert_eq!(path, vec!["world", "odom", "base_link", "camera"]);

    let reverse = tree.get_path("camera", "world").expect("reverse path");
    assert_eq!(reverse, vec!["camera", "base_link", "odom", "world"]);

    let same = tree.get_path("world", "world").expect("same frame path");
    assert_eq!(same, vec!["world"]);

    assert!(tree.get_path("world", "missing").is_none());

    tree.clear();
    assert_eq!(tree.frame_count(), 0);
    assert_eq!(tree.transform_count(), 0);
    assert!(tree.frame_names().is_empty());
    assert!(!tree.has_frame("world"));
    assert!(!tree.can_transform("world", "odom"));

    let mut single = TransformTree::new();
    single.register_frame::<World>("world");
    let identity = single.lookup("world", "world").expect("identity");
    approx_eq(identity.translation.x, 0.0, 1e-12);
    assert!(single.lookup("world", "missing").is_none());
}
