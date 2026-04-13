use glam::DVec3;

use concord::{
    Ecf, Enu, FrameCast, Geo, Wgs, batch_to_ecf, batch_to_enu, batch_to_wgs_from_enu, convert,
    enu_to_ned, frame_cast, ned_to_enu, sample_rotation_spline, to_ecf, to_enu, to_wgs,
};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Body;

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

#[test]
fn root_re_exports_cover_common_earth_and_frame_helpers() {
    let origin = Geo::new(52.0, 4.0, 10.0);
    let point = Wgs::new(52.0001, 4.0002, 12.0);

    let ecf: Ecf = to_ecf(point);
    let roundtrip = to_wgs(ecf);
    approx_eq(roundtrip.latitude, point.latitude, 1e-10);

    let enu = to_enu(origin, point);
    assert_eq!(enu.origin, origin);
    let back = batch_to_wgs_from_enu(&[enu]);
    assert_eq!(back.len(), 1);
    approx_eq(back[0].latitude, point.latitude, 1e-10);

    let batch_ecf = batch_to_ecf(&[point]);
    assert_eq!(batch_ecf.len(), 1);
    let batch_enu = batch_to_enu(origin, &[point]);
    assert_eq!(batch_enu.len(), 1);
}

#[test]
fn root_re_exports_cover_conversion_and_cast_helpers() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let point = Wgs::new(48.8570, 2.3530, 40.0);

    let enu = convert(point)
        .with_ref(origin)
        .to::<Enu>()
        .build()
        .expect("enu");
    let ned = enu_to_ned(enu);
    let enu2 = ned_to_enu(ned);
    assert_eq!(enu.origin, enu2.origin);

    let via_trait: Enu = frame_cast::<Enu, _>(enu2);
    assert_eq!(via_trait.origin, origin);

    let trait_cast: Enu = <Enu as FrameCast<Enu>>::frame_cast(enu2);
    assert_eq!(trait_cast.origin, origin);
}

#[test]
fn root_re_exports_cover_spline_sampling_helpers() {
    let spline = concord::RotationSpline::<World, Body>::from_points(vec![
        concord::Rotation::<World, Body>::identity(),
        concord::Rotation::<World, Body>::from_euler_zyx(std::f64::consts::FRAC_PI_2, 0.0, 0.0),
    ]);

    let samples = sample_rotation_spline(&spline, 3);
    assert_eq!(samples.len(), 3);

    let mid = samples[1].apply(DVec3::new(1.0, 0.0, 0.0));
    assert!(mid.x > 0.0);
    assert!(mid.y > 0.0);
}
