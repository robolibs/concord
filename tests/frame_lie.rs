use glam::DVec3;

use concord::frame::{
    Rotation, RotationTangent, Transform, TransformTangent, angle, average_rotation,
    average_two_rotation, average_two_transform, axis, interpolate_rotation, interpolate_transform,
    is_approx_rotation, is_approx_transform, is_identity_rotation, is_identity_transform,
    log_rotation, log_transform, rotation_exp, slerp, transform_exp,
};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Body;

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

fn approx_vec(a: DVec3, b: DVec3, eps: f64) {
    assert!((a - b).length() < eps, "left={a:?}, right={b:?}");
}

#[test]
fn rotation_exp_log_and_roundtrips_work() {
    let omega = RotationTangent::new(0.0, 0.0, std::f64::consts::FRAC_PI_2);
    let rot = rotation_exp::<World, Body, f64>(omega);
    let result = rot.apply(DVec3::new(1.0, 0.0, 0.0));
    approx_vec(result, DVec3::new(0.0, 1.0, 0.0), 1e-10);

    let identity = Rotation::<World, Body>::identity();
    approx_vec(log_rotation(identity), DVec3::ZERO, 1e-10);

    let omega2 = DVec3::new(0.21, 0.35, 0.56);
    let back = log_rotation(rotation_exp::<World, Body, f64>(omega2));
    approx_vec(back, omega2, 1e-10);

    let rot2 = Rotation::<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    let rot2_back = rotation_exp::<World, Body, f64>(log_rotation(rot2));
    assert!(is_approx_rotation(rot2, rot2_back, 1e-10));
}

#[test]
fn transform_exp_log_and_interpolation_work() {
    let zero: TransformTangent = [0.0; 6];
    let identity = transform_exp::<World, Body, f64>(zero);
    assert!(is_identity_transform(identity, 1e-10));

    let pure_translation: TransformTangent = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0];
    let tf = transform_exp::<World, Body, f64>(pure_translation);
    approx_vec(tf.apply(DVec3::ZERO), DVec3::new(1.0, 2.0, 3.0), 1e-10);

    let twist: TransformTangent = [0.5, -0.3, 0.7, 0.1, 0.2, 0.15];
    let back = log_transform(transform_exp::<World, Body, f64>(twist));
    for i in 0..6 {
        approx_eq(back[i], twist[i], 1e-8);
    }

    let tf1 = Transform::<World, Body>::identity();
    let tf2 = Transform::<World, Body>::from_qt(glam::DQuat::IDENTITY, DVec3::new(10.0, 0.0, 0.0));
    assert!(is_approx_transform(
        interpolate_transform(tf1, tf2, 0.0),
        tf1,
        1e-10
    ));
    assert!(is_approx_transform(
        interpolate_transform(tf1, tf2, 1.0),
        tf2,
        1e-10
    ));
    approx_vec(
        interpolate_transform(tf1, tf2, 0.5).apply(DVec3::ZERO),
        DVec3::new(5.0, 0.0, 0.0),
        1e-10,
    );
}

#[test]
fn rotation_slerp_average_and_utility_helpers_work() {
    let r1 = Rotation::<World, Body>::identity();
    let r2 = Rotation::<World, Body>::from_euler_zyx(std::f64::consts::FRAC_PI_2, 0.0, 0.0);

    assert!(is_approx_rotation(slerp(r1, r2, 0.0), r1, 1e-10));
    assert!(is_approx_rotation(slerp(r1, r2, 1.0), r2, 1e-10));
    let mid = slerp(r1, r2, 0.5);
    approx_vec(
        mid.apply(DVec3::new(1.0, 0.0, 0.0)),
        DVec3::new(
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
        ),
        1e-10,
    );

    assert!(is_approx_rotation(
        interpolate_rotation(r1, r2, 0.3),
        slerp(r1, r2, 0.3),
        1e-10
    ));
    assert!(is_approx_rotation(
        average_two_rotation(r1, r2),
        slerp(r1, r2, 0.5),
        1e-10
    ));

    let one = Rotation::<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    let averaged = average_rotation(&[one]).expect("average single");
    assert!(is_approx_rotation(averaged, one, 1e-10));
    let averaged_same = average_rotation(&[one, one, one]).expect("average identical");
    assert!(is_approx_rotation(averaged_same, one, 1e-10));

    approx_eq(angle(Rotation::<World, Body>::identity()), 0.0, 1e-10);
    approx_eq(angle(r2), std::f64::consts::FRAC_PI_2, 1e-10);
    let ax = axis(Rotation::<World, Body>::from_euler_zyx(0.5, 0.0, 0.0));
    approx_eq(ax.x, 0.0, 1e-10);
    approx_eq(ax.y, 0.0, 1e-10);
    approx_eq(ax.z.abs(), 1.0, 1e-10);

    assert!(is_identity_rotation(
        Rotation::<World, Body>::identity(),
        1e-10
    ));
    assert!(!is_identity_rotation(
        Rotation::<World, Body>::from_euler_zyx(0.1, 0.0, 0.0),
        1e-10
    ));
}

#[test]
fn transform_average_and_identity_helpers_work() {
    let tf1 = Transform::<World, Body>::identity();
    let tf2 = Transform::<World, Body>::from_qt(glam::DQuat::IDENTITY, DVec3::new(10.0, 0.0, 0.0));

    let avg = average_two_transform(tf1, tf2);
    let mid = interpolate_transform(tf1, tf2, 0.5);
    assert!(is_approx_transform(avg, mid, 1e-10));

    assert!(is_identity_transform(
        Transform::<World, Body>::identity(),
        1e-10
    ));
    assert!(!is_identity_transform(tf2, 1e-10));
}
