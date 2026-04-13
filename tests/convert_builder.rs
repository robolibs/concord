use concord::{
    Error, Geo, Wgs, convert,
    frame::{Enu, Ned},
};

fn approx_eq(a: f64, b: f64, eps: f64) {
    assert!((a - b).abs() < eps, "left={a}, right={b}");
}

#[test]
fn convert_builder_errors_without_ref_for_wgs_to_enu() {
    let wgs = Wgs::new(48.8566, 2.3522, 35.0);
    let enu = convert(wgs).to::<Enu>().build();

    assert!(matches!(enu, Err(Error::InvalidArgument(_))));
}

#[test]
fn convert_builder_wgs_to_enu_with_ref() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let wgs = Wgs::new(48.8570, 2.3530, 40.0);

    let enu = convert(wgs)
        .with_ref(origin)
        .to::<Enu>()
        .build()
        .expect("valid enu");
    assert_eq!(enu.origin, origin);
}

#[test]
fn convert_builder_enu_to_wgs_without_ref() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let original = Wgs::new(48.8570, 2.3530, 40.0);

    let enu = convert(original)
        .with_ref(origin)
        .to::<Enu>()
        .build()
        .expect("enu");

    let back = convert(enu).to::<Wgs>().build().expect("wgs");
    approx_eq(back.latitude, original.latitude, 1e-10);
    approx_eq(back.longitude, original.longitude, 1e-10);
    approx_eq(back.altitude, original.altitude, 1e-5);
}

#[test]
fn convert_builder_chaining_wgs_enu_ned_wgs() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let wgs = Wgs::new(48.8570, 2.3530, 40.0);

    let roundtrip = convert(wgs)
        .with_ref(origin)
        .to::<Enu>()
        .to::<Ned>()
        .to::<Wgs>()
        .build()
        .expect("roundtrip");

    approx_eq(roundtrip.latitude, wgs.latitude, 1e-10);
    approx_eq(roundtrip.longitude, wgs.longitude, 1e-10);
    approx_eq(roundtrip.altitude, wgs.altitude, 1e-5);
}

#[test]
fn convert_builder_with_datum_alias_and_wgs_ecf_wgs_chain() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let wgs = Wgs::new(48.8570, 2.3530, 40.0);

    let enu = convert(wgs)
        .with_datum(origin)
        .to::<Enu>()
        .build()
        .expect("enu");
    assert_eq!(enu.origin, origin);

    let original = Wgs::new(48.8566, 2.3522, 35.0);
    let roundtrip = convert(original)
        .to::<concord::Ecf>()
        .to::<Wgs>()
        .build()
        .expect("ecf roundtrip");

    approx_eq(roundtrip.latitude, original.latitude, 1e-10);
    approx_eq(roundtrip.longitude, original.longitude, 1e-10);
    approx_eq(roundtrip.altitude, original.altitude, 1e-6);
}
