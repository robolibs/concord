use concord::{Geo, Wgs, convert, frame::Enu};

fn main() {
    let origin = Geo::new(52.0, 4.0, 10.0);
    let point = Wgs::new(52.0001, 4.0002, 12.0);

    let enu = convert(point)
        .with_ref(origin)
        .to::<Enu>()
        .build()
        .expect("valid ENU conversion");

    println!(
        "ENU: east={:.3} north={:.3} up={:.3}",
        enu.east(),
        enu.north(),
        enu.up()
    );
}
