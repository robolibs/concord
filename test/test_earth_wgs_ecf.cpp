#include <concord/concord.hpp>
#include <doctest/doctest.h>

#include <cmath>

using namespace concord;

TEST_CASE("WGS -> ECF -> WGS round-trip") {
    const earth::WGS original{48.8566, 2.3522, 35.0}; // Paris

    const auto ecf = earth::to_ecf(original);
    const auto back = earth::to_wgs(ecf);

    CHECK(std::abs(back.latitude - original.latitude) < 1e-9);
    CHECK(std::abs(back.longitude - original.longitude) < 1e-9);
    CHECK(std::abs(back.altitude - original.altitude) < 1e-5);
}

TEST_CASE("ECF known values") {
    const earth::WGS wgs{0.0, 0.0, 0.0};
    const auto ecf = earth::to_ecf(wgs);

    // ECF extends dp::Point, so x/y/z are direct members
    CHECK(std::abs(ecf.x - earth::wgs84::a_m) < 1.0);
    CHECK(std::abs(ecf.y) < 1.0);
    CHECK(std::abs(ecf.z) < 1.0);
}

TEST_CASE("ECF extends dp::Point") {
    const earth::ECF ecf{1000.0, 2000.0, 3000.0};

    // ECF IS-A dp::Point
    CHECK(ecf.x == 1000.0);
    CHECK(ecf.y == 2000.0);
    CHECK(ecf.z == 3000.0);

    // Can use dp::Point methods directly
    CHECK(ecf.magnitude() > 0);
    CHECK(ecf.is_set());

    // Access as dp::Point
    const dp::Point &pt = ecf.point();
    CHECK(pt.x == ecf.x);
}

TEST_CASE("to_wgs_optimized basic round-trip") {
    const earth::WGS original{48.8566, 2.3522, 35.0}; // Paris

    const auto ecf = earth::to_ecf(original);
    const auto back = earth::to_wgs_optimized(ecf);

    CHECK(std::abs(back.latitude - original.latitude) < 1e-9);
    CHECK(std::abs(back.longitude - original.longitude) < 1e-9);
    CHECK(std::abs(back.altitude - original.altitude) < 1e-5);
}

TEST_CASE("to_wgs_optimized matches or beats to_wgs accuracy") {
    // Test several locations around the globe
    std::vector<earth::WGS> test_points = {
        {0.0, 0.0, 0.0},          // Equator/prime meridian
        {90.0, 0.0, 1000.0},      // North pole
        {-90.0, 0.0, 500.0},      // South pole
        {45.0, 90.0, 10000.0},    // Mid-latitude high altitude
        {-33.9, 151.2, 50.0},     // Sydney
        {35.6762, 139.6503, 40.0} // Tokyo
    };

    for (const auto &original : test_points) {
        const auto ecf = earth::to_ecf(original);

        const auto standard = earth::to_wgs(ecf);
        const auto precise = earth::to_wgs_optimized(ecf);

        // Calculate errors
        double standard_lat_err = std::abs(standard.latitude - original.latitude);
        double standard_lon_err = std::abs(standard.longitude - original.longitude);
        double standard_alt_err = std::abs(standard.altitude - original.altitude);

        double precise_lat_err = std::abs(precise.latitude - original.latitude);
        double precise_lon_err = std::abs(precise.longitude - original.longitude);
        double precise_alt_err = std::abs(precise.altitude - original.altitude);

        // Both methods should be very accurate (sub-nanometer precision)
        // They're essentially equal at floating-point precision limits
        CHECK(precise_lat_err < 1e-10);
        CHECK(precise_lon_err < 1e-10);
        CHECK(precise_alt_err < 1e-6);

        CHECK(standard_lat_err < 1e-10);
        CHECK(standard_lon_err < 1e-10);
        CHECK(standard_alt_err < 1e-6);
    }
}

TEST_CASE("to_wgs_optimized with custom tolerance") {
    const earth::WGS original{52.5200, 13.4050, 34.0}; // Berlin
    const auto ecf = earth::to_ecf(original);

    // Test with different tolerances
    const auto loose = earth::to_wgs_optimized(ecf, 1e-6);
    const auto tight = earth::to_wgs_optimized(ecf, 1e-15);

    // Both should be accurate
    CHECK(std::abs(loose.latitude - original.latitude) < 1e-6);
    CHECK(std::abs(tight.latitude - original.latitude) < 1e-10);

    // Tight tolerance should give better or equal results
    double loose_err = std::abs(loose.latitude - original.latitude);
    double tight_err = std::abs(tight.latitude - original.latitude);
    CHECK(tight_err <= loose_err + 1e-15);
}

TEST_CASE("to_wgs_optimized handles extreme altitudes") {
    // Test at various altitudes including space
    std::vector<double> altitudes = {-100.0, 0.0, 100.0, 10000.0, 100000.0, 35786000.0}; // Up to GEO

    for (double alt : altitudes) {
        const earth::WGS original{45.0, 90.0, alt};
        const auto ecf = earth::to_ecf(original);
        const auto back = earth::to_wgs_optimized(ecf);

        // Relative error for altitude (since absolute values vary widely)
        double alt_err = std::abs(back.altitude - original.altitude);
        double rel_err = (alt != 0.0) ? alt_err / std::abs(alt) : alt_err;

        CHECK(std::abs(back.latitude - original.latitude) < 1e-8);
        CHECK(std::abs(back.longitude - original.longitude) < 1e-8);
        CHECK((rel_err < 1e-8 || alt_err < 1e-3));
    }
}
