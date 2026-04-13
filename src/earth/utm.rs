use crate::{
    core::{Error, Result},
    earth::{
        Utm, Wgs,
        types::wgs_from_radians,
        wgs84::{A_M, DEG_TO_RAD, E2, E4, E6, EP2, K0},
    },
};

pub fn utm_zone(lon_deg: f64) -> i32 {
    ((lon_deg + 180.0) / 6.0).floor() as i32 + 1
}

pub fn is_north(lat_deg: f64) -> bool {
    lat_deg >= 0.0
}

pub fn to_utm(wgs: Wgs) -> Result<Utm> {
    if !(-80.0..=84.0).contains(&wgs.latitude) {
        return Err(Error::OutOfRange(
            "latitude out of UTM bounds (-80 to 84 deg)".into(),
        ));
    }

    let lat_rad = wgs.latitude * DEG_TO_RAD;
    let lon_rad = wgs.longitude * DEG_TO_RAD;

    let zone = utm_zone(wgs.longitude);
    if !(1..=60).contains(&zone) {
        return Err(Error::OutOfRange("UTM zone out of range [1,60]".into()));
    }

    let lon0 = (((zone - 1) * 6 - 180 + 3) as f64) * DEG_TO_RAD;

    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let tan_lat = lat_rad.tan();

    let n = A_M / (1.0 - E2 * sin_lat * sin_lat).sqrt();
    let t = tan_lat * tan_lat;
    let c = EP2 * cos_lat * cos_lat;
    let a = cos_lat * (lon_rad - lon0);

    let m = A_M
        * ((1.0 - E2 / 4.0 - 3.0 * E4 / 64.0 - 5.0 * E6 / 256.0) * lat_rad
            - (3.0 * E2 / 8.0 + 3.0 * E4 / 32.0 + 45.0 * E6 / 1024.0) * (2.0 * lat_rad).sin()
            + (15.0 * E4 / 256.0 + 45.0 * E6 / 1024.0) * (4.0 * lat_rad).sin()
            - (35.0 * E6 / 3072.0) * (6.0 * lat_rad).sin());

    let a2 = a * a;
    let a4 = a2 * a2;
    let a6 = a4 * a2;

    let easting = K0
        * n
        * (a + (1.0 - t + c) * a2 * a / 6.0
            + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * EP2) * a4 * a / 120.0)
        + 500_000.0;

    let mut northing = K0
        * (m + n
            * tan_lat
            * (a2 / 2.0
                + (5.0 - t + 9.0 * c + 4.0 * c * c) * a4 / 24.0
                + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * EP2) * a6 / 720.0));

    if wgs.latitude < 0.0 {
        northing += 10_000_000.0;
    }

    Ok(Utm {
        zone,
        band: if is_north(wgs.latitude) { 'N' } else { 'S' },
        easting,
        northing,
        altitude: wgs.altitude,
    })
}

pub fn to_wgs(utm: Utm) -> Result<Wgs> {
    if !(1..=60).contains(&utm.zone) {
        return Err(Error::OutOfRange("UTM zone out of range [1,60]".into()));
    }

    let mut northing = utm.northing;
    if utm.band < 'N' {
        northing -= 10_000_000.0;
    }

    let lon0 = (((utm.zone - 1) * 6 - 180 + 3) as f64) * DEG_TO_RAD;
    let m = northing / K0;
    let mu = m / (A_M * (1.0 - E2 / 4.0 - 3.0 * E4 / 64.0 - 5.0 * E6 / 256.0));

    let e1 = (1.0 - (1.0 - E2).sqrt()) / (1.0 + (1.0 - E2).sqrt());
    let e1_2 = e1 * e1;
    let e1_3 = e1_2 * e1;
    let e1_4 = e1_3 * e1;

    let phi1 = mu
        + (3.0 * e1 / 2.0 - 27.0 * e1_3 / 32.0) * (2.0 * mu).sin()
        + (21.0 * e1_2 / 16.0 - 55.0 * e1_4 / 32.0) * (4.0 * mu).sin()
        + (151.0 * e1_3 / 96.0) * (6.0 * mu).sin()
        + (1097.0 * e1_4 / 512.0) * (8.0 * mu).sin();

    let sin_phi1 = phi1.sin();
    let cos_phi1 = phi1.cos();
    let tan_phi1 = phi1.tan();

    let n1 = A_M / (1.0 - E2 * sin_phi1 * sin_phi1).sqrt();
    let t1 = tan_phi1 * tan_phi1;
    let c1 = EP2 * cos_phi1 * cos_phi1;
    let r1 = A_M * (1.0 - E2) / (1.0 - E2 * sin_phi1 * sin_phi1).powf(1.5);
    let d = (utm.easting - 500_000.0) / (n1 * K0);

    let d2 = d * d;
    let d4 = d2 * d2;
    let d6 = d4 * d2;

    let lat = phi1
        - (n1 * tan_phi1 / r1)
            * (d2 / 2.0 - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1 * c1 - 9.0 * EP2) * d4 / 24.0
                + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1 * t1 - 252.0 * EP2 - 3.0 * c1 * c1)
                    * d6
                    / 720.0);

    let lon = lon0
        + (d - (1.0 + 2.0 * t1 + c1) * d2 * d / 6.0
            + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1 * c1 + 8.0 * EP2 + 24.0 * t1 * t1) * d4 * d
                / 120.0)
            / cos_phi1;

    Ok(wgs_from_radians(lat, lon, utm.altitude))
}
