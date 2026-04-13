use crate::earth::{
    Ecf, Wgs,
    wgs84::{DEG_TO_RAD, E2, RAD_TO_DEG, prime_vertical_radius},
};

pub fn to_ecf(wgs: Wgs) -> Ecf {
    let lat_rad = wgs.latitude * DEG_TO_RAD;
    let lon_rad = wgs.longitude * DEG_TO_RAD;

    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    let n = prime_vertical_radius(sin_lat);
    let alt = wgs.altitude;

    Ecf::new(
        (n + alt) * cos_lat * cos_lon,
        (n + alt) * cos_lat * sin_lon,
        (n * (1.0 - E2) + alt) * sin_lat,
    )
}

pub fn to_wgs(ecf: Ecf) -> Wgs {
    to_wgs_with_tolerance(ecf, 1e-15, 20)
}

pub fn to_wgs_optimized(ecf: Ecf, tolerance: f64) -> Wgs {
    let tolerance = tolerance.max(1e-15);
    to_wgs_with_tolerance(ecf, tolerance, 50)
}

fn to_wgs_with_tolerance(ecf: Ecf, tolerance: f64, max_iterations: usize) -> Wgs {
    let x = ecf.x;
    let y = ecf.y;
    let z = ecf.z;

    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();

    if p.abs() < tolerance {
        let lat = if z >= 0.0 {
            std::f64::consts::FRAC_PI_2
        } else {
            -std::f64::consts::FRAC_PI_2
        };
        let alt = z.abs() - crate::earth::wgs84::B_M;
        return Wgs::new(lat * RAD_TO_DEG, lon * RAD_TO_DEG, alt);
    }

    let mut lat = z.atan2(p * (1.0 - E2));
    let mut n: f64;
    let mut alt = 0.0;

    for _ in 0..max_iterations {
        let prev_lat = lat;
        let sin_lat = lat.sin();
        let cos_lat = lat.cos();

        n = prime_vertical_radius(sin_lat);
        alt = if cos_lat.abs() > tolerance {
            p / cos_lat - n
        } else {
            z.signum() * z.abs() - n * (1.0 - E2)
        };

        lat = z.atan2(p * (1.0 - E2 * n / (n + alt)));

        if (lat - prev_lat).abs() < tolerance {
            break;
        }
    }

    Wgs::new(lat * RAD_TO_DEG, lon * RAD_TO_DEG, alt)
}
