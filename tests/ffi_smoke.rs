use std::ffi::CStr;

use concord::ffi::{
    ConcordGeo3, ConcordQuat, ConcordTransform, ConcordTransformTree, ConcordUtm, ConcordVec3,
    concord_convert_wgs_to_enu, concord_last_error_message, concord_transform_tree_can_transform,
    concord_transform_tree_frame_count, concord_transform_tree_free, concord_transform_tree_lookup,
    concord_transform_tree_new, concord_transform_tree_register_frame,
    concord_transform_tree_set_transform, concord_transform_tree_transform_count,
    concord_utm_to_wgs, concord_wgs_to_utm,
};

fn c_string(bytes: &'static [u8]) -> *const std::ffi::c_char {
    bytes.as_ptr().cast()
}

fn last_error() -> String {
    let ptr = concord_last_error_message();
    if ptr.is_null() {
        return String::new();
    }
    unsafe { CStr::from_ptr(ptr) }
        .to_string_lossy()
        .into_owned()
}

#[test]
fn ffi_exposes_coordinate_conversion_roundtrip() {
    let wgs = ConcordGeo3 {
        latitude: 52.0,
        longitude: 4.0,
        altitude: 10.0,
    };
    let mut utm = ConcordUtm {
        zone: 0,
        band: 0,
        easting: 0.0,
        northing: 0.0,
        altitude: 0.0,
    };

    assert!(concord_wgs_to_utm(wgs, &mut utm));
    assert_eq!(utm.zone, 31);
    assert_eq!(char::from_u32(utm.band).expect("band"), 'N');

    let mut roundtrip = ConcordGeo3 {
        latitude: 0.0,
        longitude: 0.0,
        altitude: 0.0,
    };
    assert!(concord_utm_to_wgs(utm, &mut roundtrip));
    assert!((roundtrip.latitude - wgs.latitude).abs() < 1e-6);
    assert!((roundtrip.longitude - wgs.longitude).abs() < 1e-6);
}

#[test]
fn ffi_exposes_convert_builder_and_transform_tree() {
    let world = c_string(b"world\0");
    let base = c_string(b"base\0");
    let sensor = c_string(b"sensor\0");

    let tree: *mut ConcordTransformTree = concord_transform_tree_new();
    assert!(!tree.is_null());

    assert!(concord_transform_tree_register_frame(tree, world));
    assert!(concord_transform_tree_register_frame(tree, base));
    assert!(concord_transform_tree_register_frame(tree, sensor));
    assert_eq!(concord_transform_tree_frame_count(tree), 3);

    let world_to_base = ConcordTransform {
        rotation: ConcordQuat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        translation: ConcordVec3 {
            x: 1.0,
            y: 2.0,
            z: 0.0,
        },
    };
    let base_to_sensor = ConcordTransform {
        rotation: ConcordQuat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        translation: ConcordVec3 {
            x: 0.0,
            y: 0.0,
            z: 3.0,
        },
    };

    assert!(concord_transform_tree_set_transform(
        tree,
        world,
        base,
        world_to_base
    ));
    assert!(concord_transform_tree_set_transform(
        tree,
        base,
        sensor,
        base_to_sensor
    ));
    assert_eq!(concord_transform_tree_transform_count(tree), 2);
    assert!(concord_transform_tree_can_transform(tree, world, sensor));

    let mut lookup = ConcordTransform {
        rotation: ConcordQuat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        translation: ConcordVec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    };
    assert!(concord_transform_tree_lookup(
        tree,
        world,
        sensor,
        &mut lookup
    ));
    assert_eq!(lookup.translation.x, 1.0);
    assert_eq!(lookup.translation.y, 2.0);
    assert_eq!(lookup.translation.z, 3.0);

    concord_transform_tree_free(tree);
}

#[test]
fn ffi_reports_errors_for_invalid_arguments() {
    let mut out = ConcordGeo3 {
        latitude: 0.0,
        longitude: 0.0,
        altitude: 0.0,
    };
    let invalid = ConcordUtm {
        zone: 0,
        band: 'N' as u32,
        easting: 0.0,
        northing: 0.0,
        altitude: 0.0,
    };
    assert!(!concord_utm_to_wgs(invalid, &mut out));
    assert!(last_error().contains("UTM zone out of range"));

    let origin = ConcordGeo3 {
        latitude: 52.0,
        longitude: 4.0,
        altitude: 10.0,
    };
    let point = ConcordGeo3 {
        latitude: 52.0001,
        longitude: 4.0002,
        altitude: 12.0,
    };
    let mut enu = concord::ffi::ConcordEnuPoint {
        east: 0.0,
        north: 0.0,
        up: 0.0,
        origin,
    };
    assert!(concord_convert_wgs_to_enu(point, origin, &mut enu));
    assert!(enu.east != 0.0 || enu.north != 0.0);
}
