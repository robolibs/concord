pub type Geo = datapod::Geo;
pub type Wgs = datapod::Geo;
pub type Ecf = datapod::Point;
pub type Utm = datapod::Utm;

pub fn lat_rad(value: Geo) -> f64 {
    value.latitude * crate::earth::wgs84::DEG_TO_RAD
}

pub fn lon_rad(value: Geo) -> f64 {
    value.longitude * crate::earth::wgs84::DEG_TO_RAD
}

pub fn wgs_from_radians(latitude: f64, longitude: f64, altitude: f64) -> Wgs {
    Wgs::new(
        latitude * crate::earth::wgs84::RAD_TO_DEG,
        longitude * crate::earth::wgs84::RAD_TO_DEG,
        altitude,
    )
}
