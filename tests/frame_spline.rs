use glam::{DQuat, DVec3};

use concord::frame::{
    Rotation, RotationSpline, TimedRotationSpline, TimedTransformSpline, Transform,
    TransformSpline, angle, is_approx_rotation, is_approx_transform, is_identity_rotation,
    is_identity_transform, sample_rotation_spline, sample_transform_spline,
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
fn rotation_spline_handles_empty_single_and_constructor_cases() {
    let mut empty = RotationSpline::<World, Body>::new();
    assert_eq!(empty.size(), 0);
    assert!(!empty.is_built());
    assert!(is_identity_rotation(empty.evaluate(0.0), 1e-12));

    empty.add_point(Rotation::<World, Body>::from_euler_zyx(0.5, 0.0, 0.0));
    empty.build();
    assert_eq!(empty.size(), 1);
    assert!(!empty.is_built());

    let constructed = RotationSpline::from_points(vec![
        Rotation::<World, Body>::identity(),
        Rotation::<World, Body>::from_euler_zyx(0.5, 0.0, 0.0),
        Rotation::<World, Body>::from_euler_zyx(1.0, 0.0, 0.0),
    ]);
    assert_eq!(constructed.size(), 3);
    assert!(constructed.is_built());
}

#[test]
fn rotation_spline_evaluates_endpoints_and_monotonic_angles() {
    let r1 = Rotation::<World, Body>::identity();
    let r2 = Rotation::<World, Body>::from_euler_zyx(std::f64::consts::FRAC_PI_4, 0.0, 0.0);
    let r3 = Rotation::<World, Body>::from_euler_zyx(std::f64::consts::FRAC_PI_2, 0.0, 0.0);

    let spline = RotationSpline::from_points(vec![r1, r2, r3]);
    assert!(is_approx_rotation(spline.evaluate(0.0), r1, 1e-10));
    assert!(is_approx_rotation(spline.evaluate(2.0), r3, 1e-10));
    assert!(is_approx_rotation(
        spline.evaluate_normalized(0.0),
        r1,
        1e-10
    ));
    assert!(is_approx_rotation(
        spline.evaluate_normalized(1.0),
        r3,
        1e-10
    ));

    let mut prev_angle = 0.0;
    for i in 0..=10 {
        let rot = spline.evaluate_normalized(i as f64 / 10.0);
        let ang = angle(rot);
        assert!(ang >= prev_angle - 1e-10, "angle regressed at sample {i}");
        prev_angle = ang;
    }
}

#[test]
fn transform_spline_interpolates_translation_and_samples() {
    let tf1 = Transform::<World, Body>::identity();
    let tf2 = Transform::<World, Body>::from_qt(DQuat::IDENTITY, DVec3::new(10.0, 0.0, 0.0));

    let empty = TransformSpline::<World, Body>::new();
    assert!(is_identity_transform(empty.evaluate(0.0), 1e-12));

    let spline = TransformSpline::from_points(vec![tf1, tf2]);
    assert!(spline.is_built());
    assert!(is_approx_transform(spline.evaluate(0.0), tf1, 1e-10));
    assert!(is_approx_transform(spline.evaluate(1.0), tf2, 1e-10));

    let mid = spline.evaluate_normalized(0.5);
    approx_vec(mid.apply(DVec3::ZERO), DVec3::new(5.0, 0.0, 0.0), 1e-10);

    let samples = sample_transform_spline(&spline, 5);
    assert_eq!(samples.len(), 5);
    assert!(is_approx_transform(samples[0], tf1, 1e-10));
    assert!(is_approx_transform(samples[4], tf2, 1e-10));
}

#[test]
fn timed_rotation_spline_sorts_range_checks_and_evaluates() {
    let r1 = Rotation::<World, Body>::identity();
    let r2 = Rotation::<World, Body>::from_euler_zyx(0.5, 0.0, 0.0);
    let r3 = Rotation::<World, Body>::from_euler_zyx(1.0, 0.0, 0.0);

    let mut spline = TimedRotationSpline::<World, Body>::new();
    assert_eq!(spline.size(), 0);
    assert!(!spline.can_evaluate(0.0));
    assert!(is_identity_rotation(spline.evaluate_at(0.0), 1e-12));

    spline.add_point(3.0, r3);
    spline.add_point(1.0, r1);
    spline.add_point(2.0, r2);
    spline.build();

    assert!(spline.is_built());
    assert_eq!(spline.time_range(), Some((1.0, 3.0)));
    assert!(spline.can_evaluate(1.0));
    assert!(spline.can_evaluate(2.5));
    assert!(!spline.can_evaluate(0.5));
    assert!(!spline.can_evaluate(3.5));

    assert!(is_approx_rotation(spline.evaluate_at(1.0), r1, 1e-10));
    assert!(is_approx_rotation(spline.evaluate_at(3.0), r3, 1e-10));
    let mid = spline.evaluate_at(2.5);
    assert!(angle(mid) > angle(r2));
    assert!(angle(mid) < angle(r3));
}

#[test]
fn timed_transform_spline_evaluates_midpoint_and_clamps() {
    let tf1 = Transform::<World, Body>::from_qt(DQuat::IDENTITY, DVec3::ZERO);
    let tf2 = Transform::<World, Body>::from_qt(DQuat::IDENTITY, DVec3::new(10.0, 0.0, 0.0));

    let mut spline = TimedTransformSpline::<World, Body>::new();
    spline.add_point(1.0, tf1);
    spline.add_point(2.0, tf2);
    spline.build();

    let range = spline.time_range().expect("time range");
    approx_eq(range.0, 1.0, 1e-12);
    approx_eq(range.1, 2.0, 1e-12);

    approx_vec(
        spline.evaluate_at(1.5).apply(DVec3::ZERO),
        DVec3::new(5.0, 0.0, 0.0),
        1e-10,
    );
    assert!(is_approx_transform(spline.evaluate_at(0.0), tf1, 1e-10));
    assert!(is_approx_transform(spline.evaluate_at(10.0), tf2, 1e-10));
}

#[test]
fn sample_rotation_spline_hits_endpoints() {
    let r1 = Rotation::<World, Body>::identity();
    let r2 = Rotation::<World, Body>::from_euler_zyx(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
    let spline = RotationSpline::from_points(vec![r1, r2]);

    let samples = sample_rotation_spline(&spline, 5);
    assert_eq!(samples.len(), 5);
    assert!(is_approx_rotation(samples[0], r1, 1e-10));
    assert!(is_approx_rotation(samples[4], r2, 1e-10));
}
