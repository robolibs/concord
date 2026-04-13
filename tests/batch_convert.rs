use concord::{
    Geo, Wgs,
    earth::{
        batch_to_ecf, batch_to_enu, batch_to_ned, batch_to_utm, batch_to_wgs,
        batch_to_wgs_from_enu, batch_to_wgs_from_ned, to_ecf, to_utm,
    },
};

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!(
        (a - b).abs() < eps,
        "left={a}, right={b}, diff={}",
        (a - b).abs()
    );
}

#[test]
fn batch_to_ecf_matches_scalar() {
    let points = vec![
        Wgs::new(48.8566, 2.3522, 35.0),
        Wgs::new(51.5074, -0.1278, 11.0),
        Wgs::new(40.7128, -74.0060, 10.0),
        Wgs::new(35.6762, 139.6503, 40.0),
    ];

    let out = batch_to_ecf(&points);
    assert_eq!(out.len(), points.len());

    for (actual, point) in out.iter().zip(points.iter().copied()) {
        let expected = to_ecf(point);
        approx_eq(actual.x, expected.x, 1e-9);
        approx_eq(actual.y, expected.y, 1e-9);
        approx_eq(actual.z, expected.z, 1e-9);
    }
}

#[test]
fn batch_to_utm_matches_scalar() {
    let points = vec![
        Wgs::new(48.8566, 2.3522, 35.0),
        Wgs::new(51.5074, -0.1278, 11.0),
        Wgs::new(40.7128, -74.0060, 10.0),
    ];

    let out = batch_to_utm(&points);
    assert_eq!(out.len(), points.len());

    for (actual, point) in out.iter().zip(points.iter().copied()) {
        let actual = actual.as_ref().expect("valid batch utm");
        let expected = to_utm(point).expect("valid scalar utm");
        assert_eq!(actual.zone, expected.zone);
        approx_eq(actual.easting, expected.easting, 0.01);
        approx_eq(actual.northing, expected.northing, 0.01);
    }
}

#[test]
fn batch_to_enu_and_ned_basic_behavior() {
    let origin = Geo::new(52.5200, 13.4050, 34.0);
    let points = vec![
        Wgs::new(52.5200, 13.4050, 34.0),
        Wgs::new(52.5210, 13.4050, 34.0),
        Wgs::new(52.5200, 13.4060, 34.0),
        Wgs::new(52.5200, 13.4050, 134.0),
    ];

    let enu = batch_to_enu(origin, &points);
    let ned = batch_to_ned(origin, &points);

    approx_eq(enu[0].east(), 0.0, 0.1);
    approx_eq(enu[0].north(), 0.0, 0.1);
    approx_eq(enu[0].up(), 0.0, 0.1);
    assert!(enu[1].north() > 100.0);
    assert!(enu[2].east() > 50.0);
    approx_eq(enu[3].up(), 100.0, 1.0);

    approx_eq(ned[0].north(), 0.0, 0.1);
    approx_eq(ned[0].east(), 0.0, 0.1);
    approx_eq(ned[0].down(), 0.0, 0.1);
    assert!(ned[1].north() > 100.0);
    assert!(ned[2].east() > 50.0);
    approx_eq(ned[3].down(), -100.0, 1.0);
}

#[test]
fn batch_to_wgs_from_local_frames_roundtrips() {
    let origin = Geo::new(52.5200, 13.4050, 34.0);
    let points = vec![
        Wgs::new(52.5200, 13.4050, 34.0),
        Wgs::new(52.5210, 13.4050, 34.0),
        Wgs::new(52.5200, 13.4060, 34.0),
        Wgs::new(52.5200, 13.4050, 134.0),
    ];

    let enu = batch_to_enu(origin, &points);
    let ned = batch_to_ned(origin, &points);

    let wgs_from_enu = batch_to_wgs_from_enu(&enu);
    let wgs_from_ned = batch_to_wgs_from_ned(&ned);
    let ecf = batch_to_ecf(&points);
    let wgs_from_ecf = batch_to_wgs(&ecf);

    for ((from_enu, from_ned), (from_ecf, original)) in wgs_from_enu
        .iter()
        .zip(wgs_from_ned.iter())
        .zip(wgs_from_ecf.iter().zip(points.iter()))
    {
        approx_eq(from_enu.latitude, original.latitude, 1e-8);
        approx_eq(from_enu.longitude, original.longitude, 1e-8);
        approx_eq(from_ned.latitude, original.latitude, 1e-8);
        approx_eq(from_ned.longitude, original.longitude, 1e-8);
        approx_eq(from_ecf.latitude, original.latitude, 1e-8);
        approx_eq(from_ecf.longitude, original.longitude, 1e-8);
    }
}
