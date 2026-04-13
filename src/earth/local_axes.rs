use glam::DMat3;

pub fn r_enu_from_ecf(lat_rad: f64, lon_rad: f64) -> DMat3 {
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    DMat3::from_cols_array(&[
        -sin_lon,
        cos_lon,
        0.0,
        -sin_lat * cos_lon,
        -sin_lat * sin_lon,
        cos_lat,
        cos_lat * cos_lon,
        cos_lat * sin_lon,
        sin_lat,
    ])
    .transpose()
}

pub fn r_ned_from_ecf(lat_rad: f64, lon_rad: f64) -> DMat3 {
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    DMat3::from_cols_array(&[
        -sin_lat * cos_lon,
        -sin_lat * sin_lon,
        cos_lat,
        -sin_lon,
        cos_lon,
        0.0,
        -cos_lat * cos_lon,
        -cos_lat * sin_lon,
        -sin_lat,
    ])
    .transpose()
}
