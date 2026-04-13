use concord::{Geo, Wgs, frame::Enu, frame::to_enu, frame::to_wgs_from_enu};

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

#[test]
fn enu_construction_and_origin_behavior() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let enu = Enu::new(100.0, 200.0, 50.0, origin);

    approx_eq(enu.east(), 100.0, 1e-12);
    approx_eq(enu.north(), 200.0, 1e-12);
    approx_eq(enu.up(), 50.0, 1e-12);
    assert_eq!(enu.origin, origin);
}

#[test]
fn wgs_enu_roundtrip_near_datum() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let point = Wgs::new(48.8570, 2.3530, 40.0);

    let enu = to_enu(origin, point);
    assert_eq!(enu.origin, origin);

    let back = to_wgs_from_enu(enu);
    approx_eq(back.latitude, point.latitude, 1e-10);
    approx_eq(back.longitude, point.longitude, 1e-10);
    approx_eq(back.altitude, point.altitude, 1e-6);
}

#[test]
fn enu_same_origin_and_distance_helpers() {
    let ref1 = Geo::new(48.8566, 2.3522, 35.0);
    let ref2 = Geo::new(52.5200, 13.4050, 34.0);

    let enu1 = Enu::new(100.0, 200.0, 50.0, ref1);
    let enu2 = Enu::new(150.0, 250.0, 60.0, ref1);
    let enu3 = Enu::new(100.0, 200.0, 50.0, ref2);

    assert!(enu1.same_origin(enu2));
    assert!(!enu1.same_origin(enu3));

    let a = Enu::new(0.0, 0.0, 0.0, ref1);
    let b = Enu::new(3.0, 4.0, 0.0, ref1);
    approx_eq(b.distance_from_origin(), 5.0, 1e-12);
    approx_eq(b.distance_from_origin_2d(), 5.0, 1e-12);
    approx_eq(a.distance_to(b), 5.0, 1e-12);
    approx_eq(a.distance_to_2d(b), 5.0, 1e-12);
}
