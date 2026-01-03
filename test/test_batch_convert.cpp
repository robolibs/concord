#include <concord/concord.hpp>
#include <doctest/doctest.h>

#include <cmath>
#include <vector>

using namespace concord;

// Helper to check approximate equality
constexpr double TOLERANCE = 1e-6;

bool approx_equal(double a, double b, double tol = TOLERANCE) { return std::abs(a - b) < tol; }

TEST_CASE("batch_to_ecf basic functionality") {
    // Create test points
    dp::Vector<earth::WGS> wgs_points;
    wgs_points.push_back(earth::WGS{48.8566, 2.3522, 35.0});   // Paris
    wgs_points.push_back(earth::WGS{51.5074, -0.1278, 11.0});  // London
    wgs_points.push_back(earth::WGS{40.7128, -74.0060, 10.0}); // New York
    wgs_points.push_back(earth::WGS{35.6762, 139.6503, 40.0}); // Tokyo

    // Batch convert
    auto ecf_results = earth::batch_to_ecf(std::span<const earth::WGS>(wgs_points.data(), wgs_points.size()));

    REQUIRE(ecf_results.size() == 4);

    // Verify each result matches scalar conversion exactly (batch uses same scalar code)
    for (std::size_t i = 0; i < wgs_points.size(); ++i) {
        auto expected = earth::to_ecf(wgs_points[i]);
        INFO("Point " << i << ": batch=(" << ecf_results[i].x << "," << ecf_results[i].y << "," << ecf_results[i].z
                      << ") expected=(" << expected.x << "," << expected.y << "," << expected.z << ")");
        CHECK(approx_equal(ecf_results[i].x, expected.x, 1e-9));
        CHECK(approx_equal(ecf_results[i].y, expected.y, 1e-9));
        CHECK(approx_equal(ecf_results[i].z, expected.z, 1e-9));
    }
}

TEST_CASE("batch_to_wgs basic functionality") {
    // Create test ECF points (convert from known WGS)
    dp::Vector<earth::WGS> original_wgs;
    original_wgs.push_back(earth::WGS{48.8566, 2.3522, 35.0});
    original_wgs.push_back(earth::WGS{51.5074, -0.1278, 11.0});
    original_wgs.push_back(earth::WGS{40.7128, -74.0060, 10.0});
    original_wgs.push_back(earth::WGS{35.6762, 139.6503, 40.0});

    dp::Vector<earth::ECF> ecf_points;
    for (const auto &wgs : original_wgs) {
        ecf_points.push_back(earth::to_ecf(wgs));
    }

    // Batch convert back to WGS
    auto wgs_results = earth::batch_to_wgs(std::span<const earth::ECF>(ecf_points.data(), ecf_points.size()));

    REQUIRE(wgs_results.size() == 4);

    // Verify round-trip accuracy
    for (std::size_t i = 0; i < original_wgs.size(); ++i) {
        CHECK(approx_equal(wgs_results[i].latitude, original_wgs[i].latitude, 1e-8));
        CHECK(approx_equal(wgs_results[i].longitude, original_wgs[i].longitude, 1e-8));
        CHECK(approx_equal(wgs_results[i].altitude, original_wgs[i].altitude, 1e-3));
    }
}

TEST_CASE("batch_to_ecf round-trip") {
    // Test round-trip: WGS -> ECF -> WGS
    dp::Vector<earth::WGS> original;
    original.push_back(earth::WGS{0.0, 0.0, 0.0});      // Equator/Prime meridian
    original.push_back(earth::WGS{90.0, 0.0, 0.0});     // North pole
    original.push_back(earth::WGS{-90.0, 0.0, 0.0});    // South pole
    original.push_back(earth::WGS{45.0, 90.0, 1000.0}); // Mid-latitude

    auto ecf = earth::batch_to_ecf(std::span<const earth::WGS>(original.data(), original.size()));
    auto back = earth::batch_to_wgs(std::span<const earth::ECF>(ecf.data(), ecf.size()));

    REQUIRE(back.size() == original.size());

    for (std::size_t i = 0; i < original.size(); ++i) {
        CHECK(approx_equal(back[i].latitude, original[i].latitude, 1e-8));
        CHECK(approx_equal(back[i].longitude, original[i].longitude, 1e-8));
        CHECK(approx_equal(back[i].altitude, original[i].altitude, 1e-3));
    }
}

TEST_CASE("batch_to_utm basic functionality") {
    dp::Vector<earth::WGS> wgs_points;
    wgs_points.push_back(earth::WGS{48.8566, 2.3522, 35.0});   // Paris (zone 31)
    wgs_points.push_back(earth::WGS{51.5074, -0.1278, 11.0});  // London (zone 30)
    wgs_points.push_back(earth::WGS{40.7128, -74.0060, 10.0}); // New York (zone 18)

    auto utm_results = earth::batch_to_utm(std::span<const earth::WGS>(wgs_points.data(), wgs_points.size()));

    REQUIRE(utm_results.size() == 3);

    // All should succeed (valid latitudes)
    for (std::size_t i = 0; i < utm_results.size(); ++i) {
        REQUIRE(utm_results[i].is_ok());

        // Verify against scalar conversion
        auto expected = earth::to_utm(wgs_points[i]);
        REQUIRE(expected.is_ok());

        CHECK(utm_results[i].value().zone == expected.value().zone);
        CHECK(approx_equal(utm_results[i].value().easting, expected.value().easting, 0.01));
        CHECK(approx_equal(utm_results[i].value().northing, expected.value().northing, 0.01));
    }
}

TEST_CASE("batch_to_utm out of range") {
    dp::Vector<earth::WGS> wgs_points;
    wgs_points.push_back(earth::WGS{48.8566, 2.3522, 35.0}); // Valid
    wgs_points.push_back(earth::WGS{85.0, 0.0, 0.0});        // Out of range (>84)
    wgs_points.push_back(earth::WGS{-85.0, 0.0, 0.0});       // Out of range (<-80)

    auto utm_results = earth::batch_to_utm(std::span<const earth::WGS>(wgs_points.data(), wgs_points.size()));

    REQUIRE(utm_results.size() == 3);
    CHECK(utm_results[0].is_ok());
    CHECK(utm_results[1].is_err());
    CHECK(utm_results[2].is_err());
}

TEST_CASE("batch_to_enu basic functionality") {
    // Reference point (Berlin)
    dp::Geo ref{52.5200, 13.4050, 34.0};

    // Test points around Berlin
    dp::Vector<earth::WGS> wgs_points;
    wgs_points.push_back(earth::WGS{52.5200, 13.4050, 34.0});  // At origin
    wgs_points.push_back(earth::WGS{52.5210, 13.4050, 34.0});  // ~111m north
    wgs_points.push_back(earth::WGS{52.5200, 13.4060, 34.0});  // ~70m east
    wgs_points.push_back(earth::WGS{52.5200, 13.4050, 134.0}); // 100m up

    auto enu_results = earth::batch_to_enu(ref, std::span<const earth::WGS>(wgs_points.data(), wgs_points.size()));

    REQUIRE(enu_results.size() == 4);

    // Point at origin should be (0, 0, 0)
    CHECK(approx_equal(enu_results[0].east(), 0.0, 0.1));
    CHECK(approx_equal(enu_results[0].north(), 0.0, 0.1));
    CHECK(approx_equal(enu_results[0].up(), 0.0, 0.1));

    // Point north should have positive north component
    CHECK(enu_results[1].north() > 100.0); // ~111m north

    // Point east should have positive east component
    CHECK(enu_results[2].east() > 50.0); // ~70m east

    // Point up should have positive up component
    CHECK(approx_equal(enu_results[3].up(), 100.0, 1.0));

    // All should have same origin
    for (const auto &enu : enu_results) {
        CHECK(approx_equal(enu.origin.latitude, ref.latitude, 1e-10));
        CHECK(approx_equal(enu.origin.longitude, ref.longitude, 1e-10));
    }
}

TEST_CASE("batch_to_ned basic functionality") {
    // Reference point (Berlin)
    dp::Geo ref{52.5200, 13.4050, 34.0};

    // Test points around Berlin
    dp::Vector<earth::WGS> wgs_points;
    wgs_points.push_back(earth::WGS{52.5200, 13.4050, 34.0});  // At origin
    wgs_points.push_back(earth::WGS{52.5210, 13.4050, 34.0});  // ~111m north
    wgs_points.push_back(earth::WGS{52.5200, 13.4060, 34.0});  // ~70m east
    wgs_points.push_back(earth::WGS{52.5200, 13.4050, 134.0}); // 100m up (= -100m down)

    auto ned_results = earth::batch_to_ned(ref, std::span<const earth::WGS>(wgs_points.data(), wgs_points.size()));

    REQUIRE(ned_results.size() == 4);

    // Point at origin should be (0, 0, 0)
    CHECK(approx_equal(ned_results[0].north(), 0.0, 0.1));
    CHECK(approx_equal(ned_results[0].east(), 0.0, 0.1));
    CHECK(approx_equal(ned_results[0].down(), 0.0, 0.1));

    // Point north should have positive north component
    CHECK(ned_results[1].north() > 100.0);

    // Point east should have positive east component
    CHECK(ned_results[2].east() > 50.0);

    // Point up should have negative down component
    CHECK(approx_equal(ned_results[3].down(), -100.0, 1.0));
}

TEST_CASE("batch_to_wgs from ENU") {
    dp::Geo ref{52.5200, 13.4050, 34.0};

    // Create ENU points
    dp::Vector<frame::ENU> enu_points;
    enu_points.push_back(frame::ENU{0.0, 0.0, 0.0, ref});
    enu_points.push_back(frame::ENU{100.0, 0.0, 0.0, ref}); // 100m east
    enu_points.push_back(frame::ENU{0.0, 100.0, 0.0, ref}); // 100m north
    enu_points.push_back(frame::ENU{0.0, 0.0, 100.0, ref}); // 100m up

    auto wgs_results = earth::batch_to_wgs(std::span<const frame::ENU>(enu_points.data(), enu_points.size()));

    REQUIRE(wgs_results.size() == 4);

    // Origin should be at ref
    CHECK(approx_equal(wgs_results[0].latitude, ref.latitude, 1e-8));
    CHECK(approx_equal(wgs_results[0].longitude, ref.longitude, 1e-8));
    CHECK(approx_equal(wgs_results[0].altitude, ref.altitude, 1e-3));

    // 100m east should have higher longitude
    CHECK(wgs_results[1].longitude > ref.longitude);

    // 100m north should have higher latitude
    CHECK(wgs_results[2].latitude > ref.latitude);

    // 100m up should have higher altitude
    CHECK(approx_equal(wgs_results[3].altitude, ref.altitude + 100.0, 1.0));
}

TEST_CASE("batch_to_wgs from NED") {
    dp::Geo ref{52.5200, 13.4050, 34.0};

    // Create NED points
    dp::Vector<frame::NED> ned_points;
    ned_points.push_back(frame::NED{0.0, 0.0, 0.0, ref});
    ned_points.push_back(frame::NED{100.0, 0.0, 0.0, ref});  // 100m north
    ned_points.push_back(frame::NED{0.0, 100.0, 0.0, ref});  // 100m east
    ned_points.push_back(frame::NED{0.0, 0.0, -100.0, ref}); // 100m up (negative down)

    auto wgs_results = earth::batch_to_wgs(std::span<const frame::NED>(ned_points.data(), ned_points.size()));

    REQUIRE(wgs_results.size() == 4);

    // Origin should be at ref
    CHECK(approx_equal(wgs_results[0].latitude, ref.latitude, 1e-8));
    CHECK(approx_equal(wgs_results[0].longitude, ref.longitude, 1e-8));

    // 100m north should have higher latitude
    CHECK(wgs_results[1].latitude > ref.latitude);

    // 100m east should have higher longitude
    CHECK(wgs_results[2].longitude > ref.longitude);

    // 100m up should have higher altitude
    CHECK(approx_equal(wgs_results[3].altitude, ref.altitude + 100.0, 1.0));
}

TEST_CASE("batch conversion with dp::Geo input") {
    dp::Vector<dp::Geo> geo_points;
    geo_points.push_back(dp::Geo{48.8566, 2.3522, 35.0});
    geo_points.push_back(dp::Geo{51.5074, -0.1278, 11.0});

    // Test batch_to_ecf with dp::Geo
    auto ecf_results = earth::batch_to_ecf(std::span<const dp::Geo>(geo_points.data(), geo_points.size()));
    REQUIRE(ecf_results.size() == 2);

    // Verify against scalar conversion
    auto expected0 = earth::to_ecf(earth::WGS{geo_points[0]});
    CHECK(approx_equal(ecf_results[0].x, expected0.x, 1.0));

    // Test batch_to_enu with dp::Geo
    dp::Geo ref{50.0, 0.0, 0.0};
    auto enu_results = earth::batch_to_enu(ref, std::span<const dp::Geo>(geo_points.data(), geo_points.size()));
    REQUIRE(enu_results.size() == 2);

    // Test batch_to_ned with dp::Geo
    auto ned_results = earth::batch_to_ned(ref, std::span<const dp::Geo>(geo_points.data(), geo_points.size()));
    REQUIRE(ned_results.size() == 2);
}

TEST_CASE("batch conversion empty input") {
    std::span<const earth::WGS> empty_wgs;
    std::span<const earth::ECF> empty_ecf;
    std::span<const frame::ENU> empty_enu;
    std::span<const frame::NED> empty_ned;

    auto ecf_results = earth::batch_to_ecf(empty_wgs);
    CHECK(ecf_results.empty());

    auto wgs_results = earth::batch_to_wgs(empty_ecf);
    CHECK(wgs_results.empty());

    dp::Geo ref{0.0, 0.0, 0.0};
    auto enu_results = earth::batch_to_enu(ref, empty_wgs);
    CHECK(enu_results.empty());

    auto ned_results = earth::batch_to_ned(ref, empty_wgs);
    CHECK(ned_results.empty());

    auto wgs_from_enu = earth::batch_to_wgs(empty_enu);
    CHECK(wgs_from_enu.empty());

    auto wgs_from_ned = earth::batch_to_wgs(empty_ned);
    CHECK(wgs_from_ned.empty());
}

TEST_CASE("batch conversion single element") {
    // Test that single-element batches work (no SIMD, just scalar fallback)
    dp::Vector<earth::WGS> single_wgs;
    single_wgs.push_back(earth::WGS{48.8566, 2.3522, 35.0});

    auto ecf_results = earth::batch_to_ecf(std::span<const earth::WGS>(single_wgs.data(), single_wgs.size()));
    REQUIRE(ecf_results.size() == 1);

    auto expected = earth::to_ecf(single_wgs[0]);
    CHECK(approx_equal(ecf_results[0].x, expected.x, 1.0));
    CHECK(approx_equal(ecf_results[0].y, expected.y, 1.0));
    CHECK(approx_equal(ecf_results[0].z, expected.z, 1.0));
}

TEST_CASE("batch conversion non-aligned count") {
    // Test with 7 elements (not divisible by SIMD width of 4)
    dp::Vector<earth::WGS> wgs_points;
    for (int i = 0; i < 7; ++i) {
        wgs_points.push_back(earth::WGS{40.0 + i * 1.0, -74.0 + i * 0.5, 10.0 + i * 5.0});
    }

    auto ecf_results = earth::batch_to_ecf(std::span<const earth::WGS>(wgs_points.data(), wgs_points.size()));
    REQUIRE(ecf_results.size() == 7);

    // Verify all results
    for (std::size_t i = 0; i < wgs_points.size(); ++i) {
        auto expected = earth::to_ecf(wgs_points[i]);
        CHECK(approx_equal(ecf_results[i].x, expected.x, 1.0));
        CHECK(approx_equal(ecf_results[i].y, expected.y, 1.0));
        CHECK(approx_equal(ecf_results[i].z, expected.z, 1.0));
    }
}
