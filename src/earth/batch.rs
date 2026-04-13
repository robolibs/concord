use crate::{
    core::Result,
    earth::{
        Ecf, Geo, Utm, Wgs,
        local_axes::r_ned_from_ecf,
        to_ecf, to_utm, to_wgs,
        types::{lat_rad, lon_rad},
    },
    frame::{
        Enu, Ned,
        convert::{to_wgs_from_enu, to_wgs_from_ned},
    },
    math::mat3_mul_vec,
};

pub fn batch_to_ecf(wgs_coords: &[Wgs]) -> Vec<Ecf> {
    wgs_coords.iter().copied().map(to_ecf).collect()
}

pub fn batch_to_wgs(ecf_coords: &[Ecf]) -> Vec<Wgs> {
    ecf_coords.iter().copied().map(to_wgs).collect()
}

pub fn batch_to_utm(wgs_coords: &[Wgs]) -> Vec<Result<Utm>> {
    wgs_coords.iter().copied().map(to_utm).collect()
}

pub fn batch_to_enu(origin: Geo, wgs_coords: &[Wgs]) -> Vec<Enu> {
    let origin_ecf = to_ecf(origin);
    let rotation = crate::earth::local_axes::r_enu_from_ecf(lat_rad(origin), lon_rad(origin));

    batch_to_ecf(wgs_coords)
        .into_iter()
        .map(|ecf| {
            let enu = mat3_mul_vec(rotation, ecf - origin_ecf);
            Enu::new(enu.x, enu.y, enu.z, origin)
        })
        .collect()
}

pub fn batch_to_ned(origin: Geo, wgs_coords: &[Wgs]) -> Vec<Ned> {
    let origin_ecf = to_ecf(origin);
    let rotation = r_ned_from_ecf(lat_rad(origin), lon_rad(origin));

    batch_to_ecf(wgs_coords)
        .into_iter()
        .map(|ecf| {
            let ned = mat3_mul_vec(rotation, ecf - origin_ecf);
            Ned::new(ned.x, ned.y, ned.z, origin)
        })
        .collect()
}

pub fn batch_to_wgs_from_enu(enu_coords: &[Enu]) -> Vec<Wgs> {
    enu_coords.iter().copied().map(to_wgs_from_enu).collect()
}

pub fn batch_to_wgs_from_ned(ned_coords: &[Ned]) -> Vec<Wgs> {
    ned_coords.iter().copied().map(to_wgs_from_ned).collect()
}
