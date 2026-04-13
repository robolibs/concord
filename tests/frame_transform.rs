use glam::{DQuat, DVec3};

use concord::frame::{Rotation, Transform};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Body;
#[derive(Debug, Clone, Copy)]
struct Sensor;

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

fn approx_vec(a: DVec3, b: DVec3, eps: f64) {
    assert!((a - b).length() < eps, "left={a:?}, right={b:?}");
}

#[test]
fn rotation_identity() {
    let rot = Rotation::<World, Body>::identity();
    let p = DVec3::new(1.0, 2.0, 3.0);
    approx_vec(rot.apply(p), p, 1e-12);
}

#[test]
fn rotation_from_euler_zyx() {
    let rot = Rotation::<World, Body>::from_euler_zyx(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
    let p = DVec3::new(1.0, 0.0, 0.0);
    let result = rot.apply(p);

    approx_eq(result.x, 0.0, 1e-10);
    approx_eq(result.y, 1.0, 1e-10);
    approx_eq(result.z, 0.0, 1e-10);
}

#[test]
fn rotation_inverse_and_composition() {
    let rot = Rotation::<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    let p = DVec3::new(1.0, 2.0, 3.0);
    let p_world = rot.apply(p);
    let p_back = rot.inverse().apply(p_world);
    approx_vec(p_back, p, 1e-10);

    let r_wb = Rotation::<World, Body>::from_euler_zyx(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
    let r_bs = Rotation::<Body, Sensor>::from_euler_zyx(0.0, std::f64::consts::FRAC_PI_2, 0.0);
    let r_ws = r_wb * r_bs;
    let result = r_ws.apply(DVec3::new(1.0, 0.0, 0.0));
    approx_eq(result.z, -1.0, 1e-10);
}

#[test]
fn transform_identity_translation_inverse_and_mul() {
    let tf = Transform::<World, Body>::identity();
    let p = DVec3::new(1.0, 2.0, 3.0);
    approx_vec(tf.apply(p), p, 1e-12);

    let translated =
        Transform::<World, Body>::from_qt(DQuat::IDENTITY, DVec3::new(10.0, 20.0, 30.0));
    approx_vec(
        translated.apply(DVec3::new(1.0, 0.0, 0.0)),
        DVec3::new(11.0, 20.0, 30.0),
        1e-12,
    );

    let p_body = DVec3::new(1.0, 2.0, 3.0);
    let p_world = translated.apply(p_body);
    let p_back = translated.inverse().apply(p_world);
    approx_vec(p_back, p_body, 1e-10);

    let point = translated * DVec3::new(1.0, 1.0, 1.0);
    approx_vec(point, DVec3::new(11.0, 21.0, 31.0), 1e-12);
}

#[test]
fn transform_composition() {
    let t_wb = Transform::<World, Body>::from_qt(DQuat::IDENTITY, DVec3::new(10.0, 0.0, 0.0));
    let t_bs = Transform::<Body, Sensor>::from_qt(DQuat::IDENTITY, DVec3::new(0.0, 1.0, 0.0));
    let t_ws = t_wb * t_bs;

    approx_vec(t_ws.apply(DVec3::ZERO), DVec3::new(10.0, 1.0, 0.0), 1e-12);
}
