pub const A_M: f64 = 6_378_137.0;
pub const F: f64 = 1.0 / 298.257_223_563;
pub const B_M: f64 = A_M * (1.0 - F);
pub const E2: f64 = F * (2.0 - F);
pub const EP2: f64 = E2 / (1.0 - E2);
pub const E4: f64 = E2 * E2;
pub const E6: f64 = E4 * E2;
pub const K0: f64 = 0.9996;

pub const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;
pub const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;

pub fn prime_vertical_radius(sin_lat: f64) -> f64 {
    A_M / (1.0 - E2 * sin_lat * sin_lat).sqrt()
}
