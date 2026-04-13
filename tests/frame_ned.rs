use concord::{
    Geo, Wgs,
    frame::{Enu, Ned, to_enu, to_ned, to_wgs_from_ned},
};

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

#[test]
fn ned_construction_with_origin() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let ned = Ned::new(100.0, 200.0, 50.0, origin);

    approx_eq(ned.north(), 100.0, 1e-12);
    approx_eq(ned.east(), 200.0, 1e-12);
    approx_eq(ned.down(), 50.0, 1e-12);
    assert_eq!(ned.origin, origin);
}

#[test]
fn enu_ned_conversion_preserves_origin() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let enu = Enu::new(1.0, 2.0, 3.0, origin);

    let ned = to_ned(origin, to_wgs_from_ned(Ned::new(2.0, 1.0, -3.0, origin)));
    let direct = concord::frame::enu_to_ned(enu);

    approx_eq(direct.north(), 2.0, 1e-12);
    approx_eq(direct.east(), 1.0, 1e-12);
    approx_eq(direct.down(), -3.0, 1e-12);
    assert_eq!(direct.origin, origin);

    let back = to_enu(origin, to_wgs_from_ned(direct));
    approx_eq(back.east(), enu.east(), 1e-8);
    approx_eq(back.north(), enu.north(), 1e-8);
    approx_eq(back.up(), enu.up(), 1e-8);

    assert!(ned.same_origin(direct));
}

#[test]
fn wgs_ned_roundtrip_near_datum() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let point = Wgs::new(48.8570, 2.3530, 40.0);

    let ned = to_ned(origin, point);
    assert_eq!(ned.origin, origin);

    let back = to_wgs_from_ned(ned);
    approx_eq(back.latitude, point.latitude, 1e-10);
    approx_eq(back.longitude, point.longitude, 1e-10);
    approx_eq(back.altitude, point.altitude, 1e-6);
}
