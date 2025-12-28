#include <concord/concord.hpp>
#include <doctest/doctest.h>
#include <optinum/lina/basic/transpose.hpp>

#include <cmath>

using namespace concord;
namespace on = optinum;

TEST_CASE("Matrix3d type aliases") {
    // Verify type aliases work - these are just optinum types
    earth::Matrix3d R = earth::R_enu_from_ecf(0.0, 0.0);
    CHECK(R.rows() == 3);
    CHECK(R.cols() == 3);
}

TEST_CASE("R_enu_from_ecf at equator prime meridian") {
    const auto R = earth::R_enu_from_ecf(0.0, 0.0);

    // Check that the matrix is orthogonal (R * R^T = I) using optinum
    const auto Rt = on::lina::transpose(R);
    const auto I = R * Rt;

    CHECK(I(0, 0) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(I(1, 1) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(I(2, 2) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(I(0, 1) == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(I(0, 2) == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(I(1, 2) == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("R_ned_from_ecf at equator prime meridian") {
    const auto R = earth::R_ned_from_ecf(0.0, 0.0);

    // Check that the matrix is orthogonal using optinum
    const auto Rt = on::lina::transpose(R);
    const auto I = R * Rt;

    CHECK(I(0, 0) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(I(1, 1) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(I(2, 2) == doctest::Approx(1.0).epsilon(1e-10));
}

TEST_CASE("Matrix-vector multiplication using optinum operator*") {
    // Create a simple rotation matrix (90 degrees around Z)
    earth::Matrix3d R{};
    R(0, 0) = 0.0;
    R(0, 1) = -1.0;
    R(0, 2) = 0.0;
    R(1, 0) = 1.0;
    R(1, 1) = 0.0;
    R(1, 2) = 0.0;
    R(2, 0) = 0.0;
    R(2, 1) = 0.0;
    R(2, 2) = 1.0;

    earth::Vector3d v{1.0, 0.0, 0.0};
    earth::Vector3d result = R * v; // Using optinum's operator*

    // Rotating (1,0,0) by 90 degrees around Z should give (0,1,0)
    CHECK(result[0] == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(result[1] == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(result[2] == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("ENU/NED conversion consistency") {
    // Converting a point through ENU and NED should give consistent results
    const double lat_rad = 0.5;  // ~28.6 degrees
    const double lon_rad = 0.25; // ~14.3 degrees

    const auto R_enu = earth::R_enu_from_ecf(lat_rad, lon_rad);
    const auto R_ned = earth::R_ned_from_ecf(lat_rad, lon_rad);

    // A point in ECEF - using optinum Vector3d
    earth::Vector3d p_ecef{1000.0, 2000.0, 3000.0};

    // Convert to ENU and NED using optinum's operator*
    earth::Vector3d p_enu = R_enu * p_ecef;
    earth::Vector3d p_ned = R_ned * p_ecef;

    // ENU and NED should be related by: NED = (north, east, -up) = (enu[1], enu[0], -enu[2])
    CHECK(p_ned[0] == doctest::Approx(p_enu[1]).epsilon(1e-6));  // north = enu.y
    CHECK(p_ned[1] == doctest::Approx(p_enu[0]).epsilon(1e-6));  // east = enu.x
    CHECK(p_ned[2] == doctest::Approx(-p_enu[2]).epsilon(1e-6)); // down = -up
}
