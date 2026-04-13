use concord::{
    Geo, Wgs, convert,
    frame::{Enu, Ned},
};

fn main() {
    let origin = Geo::new(48.8566, 2.3522, 35.0);
    let point = Wgs::new(48.8570, 2.3530, 40.0);

    let roundtrip = convert(point)
        .with_ref(origin)
        .to::<Enu>()
        .to::<Ned>()
        .to::<Wgs>()
        .build()
        .expect("roundtrip");

    println!(
        "Roundtrip WGS: lat={:.7} lon={:.7} alt={:.3}",
        roundtrip.latitude, roundtrip.longitude, roundtrip.altitude
    );
}
