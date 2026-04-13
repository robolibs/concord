use crate::{
    core::Result,
    earth::{Ecf, Geo, Utm, Wgs, local_axes::r_ned_from_ecf, to_ecf, to_utm, to_wgs},
    frame::{
        Enu, Ned,
        convert::{to_wgs_from_enu, to_wgs_from_ned},
    },
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
    let origin_ecf = to_ecf(origin.into()).as_dvec3();
    let rotation = crate::earth::local_axes::r_enu_from_ecf(origin.lat_rad(), origin.lon_rad());

    batch_to_ecf(wgs_coords)
        .into_iter()
        .map(|ecf| {
            let enu = rotation * (ecf.as_dvec3() - origin_ecf);
            Enu::new(enu.x, enu.y, enu.z, origin)
        })
        .collect()
}

pub fn batch_to_ned(origin: Geo, wgs_coords: &[Wgs]) -> Vec<Ned> {
    let origin_ecf = to_ecf(origin.into()).as_dvec3();
    let rotation = r_ned_from_ecf(origin.lat_rad(), origin.lon_rad());

    batch_to_ecf(wgs_coords)
        .into_iter()
        .map(|ecf| {
            let ned = rotation * (ecf.as_dvec3() - origin_ecf);
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
