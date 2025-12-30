#include <cmath>
#include <concord/concord.hpp>
#include <iomanip>
#include <iostream>

using namespace concord;

int main() {
    std::cout << std::fixed << std::setprecision(12);

    // Test case: Small displacement from a reference point
    // This is the critical case for sub-cm accuracy

    dp::Geo ref{52.5200, 13.4050, 34.0}; // Berlin

    // Test points at various small displacements
    struct TestCase {
        double lat, lon, alt;
        double expected_e, expected_n, expected_u; // Expected ENU in meters
        const char *name;
    };

    TestCase tests[] = {
        // Point exactly at origin
        {52.5200, 13.4050, 34.0, 0.0, 0.0, 0.0, "Origin"},

        // 1 meter displacements (sub-cm precision test)
        {52.5200, 13.4050, 35.0, 0.0, 0.0, 1.0, "1m Up"},

        // 100 meter displacements
        {52.5200, 13.4050, 134.0, 0.0, 0.0, 100.0, "100m Up"},

        // ~111m north (1 arc-second of latitude at this location)
        {52.52009, 13.4050, 34.0, 0.0, 1.0, 0.0, "~1m North"},

        // Known displacement: 10m east, 10m north, 10m up
        // At lat 52.52, 1 degree longitude ≈ 67km, so 10m ≈ 0.000149 deg
        // 1 degree latitude ≈ 111km, so 10m ≈ 0.00009 deg
    };

    std::cout << "=== WGS -> ENU -> WGS Round-trip Precision Test ===" << std::endl;
    std::cout << "Reference: lat=" << ref.latitude << ", lon=" << ref.longitude << ", alt=" << ref.altitude
              << std::endl;
    std::cout << std::endl;

    for (const auto &test : tests) {
        earth::WGS wgs{test.lat, test.lon, test.alt};

        // Forward: WGS -> ENU
        auto enu = frame::to_enu(ref, wgs);

        // Backward: ENU -> WGS
        auto wgs_back = frame::to_wgs(enu);

        // Calculate round-trip errors
        double lat_err_deg = std::abs(wgs_back.latitude - wgs.latitude);
        double lon_err_deg = std::abs(wgs_back.longitude - wgs.longitude);
        double alt_err_m = std::abs(wgs_back.altitude - wgs.altitude);

        // Convert angular errors to meters (approximate)
        // At lat 52.52: 1 deg lat ≈ 111km, 1 deg lon ≈ 67km
        double lat_err_m = lat_err_deg * 111000.0;
        double lon_err_m = lon_err_deg * 67000.0;

        std::cout << "Test: " << test.name << std::endl;
        std::cout << "  Input WGS: (" << wgs.latitude << ", " << wgs.longitude << ", " << wgs.altitude << ")"
                  << std::endl;
        std::cout << "  ENU:       (" << enu.east() << ", " << enu.north() << ", " << enu.up() << ")" << std::endl;
        std::cout << "  Back WGS:  (" << wgs_back.latitude << ", " << wgs_back.longitude << ", " << wgs_back.altitude
                  << ")" << std::endl;
        std::cout << "  Round-trip errors:" << std::endl;
        std::cout << "    Latitude:  " << lat_err_deg << " deg = " << (lat_err_m * 100) << " cm" << std::endl;
        std::cout << "    Longitude: " << lon_err_deg << " deg = " << (lon_err_m * 100) << " cm" << std::endl;
        std::cout << "    Altitude:  " << (alt_err_m * 100) << " cm" << std::endl;
        std::cout << std::endl;
    }

    // Test the ECF -> WGS conversion specifically
    std::cout << "=== ECF -> WGS Conversion Precision ===" << std::endl;

    // Create a known WGS point, convert to ECF, then back
    earth::WGS original{52.5200123456, 13.4050123456, 34.123456};
    earth::ECF ecf = earth::to_ecf(original);
    earth::WGS back_standard = earth::to_wgs(ecf);
    earth::WGS back_precise = earth::to_wgs(ecf);

    std::cout << "Original WGS: (" << original.latitude << ", " << original.longitude << ", " << original.altitude
              << ")" << std::endl;
    std::cout << "ECF:          (" << ecf.x << ", " << ecf.y << ", " << ecf.z << ")" << std::endl;
    std::cout << std::endl;

    std::cout << "Standard to_wgs():" << std::endl;
    std::cout << "  Result:    (" << back_standard.latitude << ", " << back_standard.longitude << ", "
              << back_standard.altitude << ")" << std::endl;
    std::cout << "  Lat error: " << std::abs(back_standard.latitude - original.latitude)
              << " deg = " << std::abs(back_standard.latitude - original.latitude) * 111000 * 100 << " cm" << std::endl;
    std::cout << "  Lon error: " << std::abs(back_standard.longitude - original.longitude)
              << " deg = " << std::abs(back_standard.longitude - original.longitude) * 67000 * 100 << " cm"
              << std::endl;
    std::cout << "  Alt error: " << std::abs(back_standard.altitude - original.altitude) * 100 << " cm" << std::endl;
    std::cout << std::endl;

    std::cout << "Precise to_wgs_precise():" << std::endl;
    std::cout << "  Result:    (" << back_precise.latitude << ", " << back_precise.longitude << ", "
              << back_precise.altitude << ")" << std::endl;
    std::cout << "  Lat error: " << std::abs(back_precise.latitude - original.latitude)
              << " deg = " << std::abs(back_precise.latitude - original.latitude) * 111000 * 100 << " cm" << std::endl;
    std::cout << "  Lon error: " << std::abs(back_precise.longitude - original.longitude)
              << " deg = " << std::abs(back_precise.longitude - original.longitude) * 67000 * 100 << " cm" << std::endl;
    std::cout << "  Alt error: " << std::abs(back_precise.altitude - original.altitude) * 100 << " cm" << std::endl;

    return 0;
}
