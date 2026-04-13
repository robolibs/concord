use glam::DVec3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geo {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

impl Geo {
    pub const fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }

    pub fn lat_rad(self) -> f64 {
        self.latitude * crate::earth::wgs84::DEG_TO_RAD
    }

    pub fn lon_rad(self) -> f64 {
        self.longitude * crate::earth::wgs84::DEG_TO_RAD
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wgs {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

impl Wgs {
    pub const fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }

    pub fn from_radians(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self::new(
            latitude * crate::earth::wgs84::RAD_TO_DEG,
            longitude * crate::earth::wgs84::RAD_TO_DEG,
            altitude,
        )
    }

    pub fn lat_rad(self) -> f64 {
        self.latitude * crate::earth::wgs84::DEG_TO_RAD
    }

    pub fn lon_rad(self) -> f64 {
        self.longitude * crate::earth::wgs84::DEG_TO_RAD
    }
}

impl From<Geo> for Wgs {
    fn from(value: Geo) -> Self {
        Self::new(value.latitude, value.longitude, value.altitude)
    }
}

impl From<Wgs> for Geo {
    fn from(value: Wgs) -> Self {
        Self::new(value.latitude, value.longitude, value.altitude)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ecf {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Ecf {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn as_dvec3(self) -> DVec3 {
        DVec3::new(self.x, self.y, self.z)
    }

    pub fn magnitude(self) -> f64 {
        self.as_dvec3().length()
    }
}

impl From<DVec3> for Ecf {
    fn from(value: DVec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<Ecf> for DVec3 {
    fn from(value: Ecf) -> Self {
        value.as_dvec3()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Utm {
    pub zone: i32,
    pub band: char,
    pub easting: f64,
    pub northing: f64,
    pub altitude: f64,
}

impl Utm {
    pub const fn new(zone: i32, band: char, easting: f64, northing: f64, altitude: f64) -> Self {
        Self {
            zone,
            band,
            easting,
            northing,
            altitude,
        }
    }

    pub fn is_northern(self) -> bool {
        self.band >= 'N'
    }

    pub fn is_north(self) -> bool {
        self.is_northern()
    }
}
