use concord::{
    Geo,
    frame::{Enu, Flu, FrameTag, FrameTraits, Frd, Ned, frame_cast},
};

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

#[test]
fn frame_cast_enu_ned_preserves_origin() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let enu = Enu::new(10.0, 20.0, 30.0, origin);

    let ned: Ned = frame_cast(enu);
    approx_eq(ned.north(), 20.0, 1e-12);
    approx_eq(ned.east(), 10.0, 1e-12);
    approx_eq(ned.down(), -30.0, 1e-12);
    assert_eq!(ned.origin, origin);

    let back: Enu = frame_cast(ned);
    approx_eq(back.east(), 10.0, 1e-12);
    approx_eq(back.north(), 20.0, 1e-12);
    approx_eq(back.up(), 30.0, 1e-12);
    assert_eq!(back.origin, origin);
}

#[test]
fn frame_cast_frd_flu_and_identity_work() {
    let frd = Frd::new(1.0, 2.0, 3.0);
    let flu: Flu = frame_cast(frd);
    approx_eq(flu.forward(), 1.0, 1e-12);
    approx_eq(flu.left(), -2.0, 1e-12);
    approx_eq(flu.up(), -3.0, 1e-12);

    let back: Frd = frame_cast(flu);
    approx_eq(back.forward(), 1.0, 1e-12);
    approx_eq(back.right(), 2.0, 1e-12);
    approx_eq(back.down(), 3.0, 1e-12);

    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let enu = Enu::new(1.0, 2.0, 3.0, origin);
    let same: Enu = frame_cast(enu);
    assert_eq!(same.origin, origin);
    approx_eq(same.east(), 1.0, 1e-12);
}

#[test]
fn semantic_accessors_and_traits_are_exposed() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);

    let enu = Enu::new(10.0, 20.0, 30.0, origin);
    approx_eq(enu.east(), 10.0, 1e-12);
    approx_eq(enu.north(), 20.0, 1e-12);
    approx_eq(enu.up(), 30.0, 1e-12);
    approx_eq(enu.x(), 10.0, 1e-12);
    approx_eq(enu.y(), 20.0, 1e-12);
    approx_eq(enu.z(), 30.0, 1e-12);

    let ned = Ned::new(10.0, 20.0, 30.0, origin);
    approx_eq(ned.north(), 10.0, 1e-12);
    approx_eq(ned.east(), 20.0, 1e-12);
    approx_eq(ned.down(), 30.0, 1e-12);

    let frd = Frd::new(3.0, 4.0, 0.0);
    approx_eq(frd.magnitude(), 5.0, 1e-12);

    let flu = Flu::new(3.0, 4.0, 0.0);
    approx_eq(flu.magnitude(), 5.0, 1e-12);

    assert_eq!(Enu::TAG, FrameTag::LocalTangent);
    assert_eq!(Ned::TAG, FrameTag::LocalTangent);
    assert_eq!(Frd::TAG, FrameTag::Body);
    assert_eq!(Flu::TAG, FrameTag::Body);

    assert_eq!(<Enu as FrameTraits>::TAG, FrameTag::LocalTangent);
    assert_eq!(<Ned as FrameTraits>::TAG, FrameTag::LocalTangent);
    assert_eq!(<Frd as FrameTraits>::TAG, FrameTag::Body);
    assert_eq!(<Flu as FrameTraits>::TAG, FrameTag::Body);
}
