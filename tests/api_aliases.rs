use std::any::TypeId;

use concord::{
    earth::{Ecf, Geo, Utm, Wgs},
    frame::{Enu, Flu, Frd, Ned},
};

#[test]
fn canonical_earth_and_frame_types_are_root_exported() {
    let origin = Geo::new(52.0, 4.0, 10.0);
    let wgs = Wgs::new(52.0, 4.0, 10.0);
    let ecf = Ecf::new(1.0, 2.0, 3.0);
    let utm = Utm {
        zone: 31,
        band: 'U',
        easting: 500_000.0,
        northing: 5_760_000.0,
        altitude: 15.0,
    };
    let enu = Enu::new(1.0, 2.0, 3.0, origin);
    let ned = Ned::new(2.0, 1.0, -3.0, origin);
    let frd = Frd::new(1.0, 2.0, 3.0);
    let flu = Flu::new(1.0, -2.0, -3.0);

    assert_eq!(TypeId::of::<concord::Geo>(), TypeId::of::<Geo>());
    assert_eq!(TypeId::of::<concord::Wgs>(), TypeId::of::<Wgs>());
    assert_eq!(TypeId::of::<concord::Ecf>(), TypeId::of::<Ecf>());
    assert_eq!(TypeId::of::<concord::Utm>(), TypeId::of::<Utm>());
    assert_eq!(TypeId::of::<concord::Enu>(), TypeId::of::<Enu>());
    assert_eq!(TypeId::of::<concord::Ned>(), TypeId::of::<Ned>());
    assert_eq!(TypeId::of::<concord::Frd>(), TypeId::of::<Frd>());
    assert_eq!(TypeId::of::<concord::Flu>(), TypeId::of::<Flu>());

    assert_eq!(wgs, concord::Wgs::new(52.0, 4.0, 10.0));
    assert_eq!(ecf, concord::Ecf::new(1.0, 2.0, 3.0));
    assert_eq!(
        utm,
        concord::Utm {
            zone: 31,
            band: 'U',
            easting: 500_000.0,
            northing: 5_760_000.0,
            altitude: 15.0,
        }
    );
    assert_eq!(enu, concord::Enu::new(1.0, 2.0, 3.0, origin));
    assert_eq!(ned, concord::Ned::new(2.0, 1.0, -3.0, origin));
    assert_eq!(frd, concord::Frd::new(1.0, 2.0, 3.0));
    assert_eq!(flu, concord::Flu::new(1.0, -2.0, -3.0));
}
