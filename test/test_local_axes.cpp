#include <concord/concord.hpp>
#include <doctest/doctest.h>

#include <cmath>

using namespace concord;

TEST_CASE("Matrix3d type aliases") {
    // Verify type aliases work
    earth::Matrix3d R = earth::R_enu_from_ecf(0.0, 0.0);
    CHECK(R.rows() == 3);
    CHECK(R.cols() == 3);
}

TEST_CASE("R_enu_from_ecf at equator prime meridian") {
    // At lat=0, lon=0, the ENU frame should be:
    // East = Y (ECEF), North = Z (ECEF), Up = X (ECEF)
    const auto R = earth::R_enu_from_ecf(0.0, 0.0);

    // Check that the matrix is orthogonal (R * R^T = I)
    const auto Rt = earth::transpose(R);
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

    // Check that the matrix is orthogonal
    const auto Rt = earth::transpose(R);
    const auto I = R * Rt;

    CHECK(I(0, 0) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(I(1, 1) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(I(2, 2) == doctest::Approx(1.0).epsilon(1e-10));
}

TEST_CASE("mat_mul with Matrix3d") {
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

    dp::Point v{1.0, 0.0, 0.0};
    dp::Point result = earth::mat_mul(R, v);

    // Rotating (1,0,0) by 90 degrees around Z should give (0,1,0)
    CHECK(result.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(result.y == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("mat_mul backward compatibility with dp::mat::matrix3x3d") {
    // Test that the overload for dp::mat::matrix3x3d still works
    dp::mat::matrix3x3d R{};
    R(0, 0) = 1.0;
    R(0, 1) = 0.0;
    R(0, 2) = 0.0;
    R(1, 0) = 0.0;
    R(1, 1) = 1.0;
    R(1, 2) = 0.0;
    R(2, 0) = 0.0;
    R(2, 1) = 0.0;
    R(2, 2) = 1.0;

    dp::Point v{1.0, 2.0, 3.0};
    dp::Point result = earth::mat_mul(R, v);

    // Identity matrix should not change the vector
    CHECK(result.x == doctest::Approx(v.x));
    CHECK(result.y == doctest::Approx(v.y));
    CHECK(result.z == doctest::Approx(v.z));
}

TEST_CASE("transpose backward compatibility with dp::mat::matrix3x3d") {
    dp::mat::matrix3x3d R{};
    R(0, 0) = 1.0;
    R(0, 1) = 2.0;
    R(0, 2) = 3.0;
    R(1, 0) = 4.0;
    R(1, 1) = 5.0;
    R(1, 2) = 6.0;
    R(2, 0) = 7.0;
    R(2, 1) = 8.0;
    R(2, 2) = 9.0;

    dp::mat::matrix3x3d Rt = earth::transpose(R);

    CHECK(Rt(0, 0) == R(0, 0));
    CHECK(Rt(0, 1) == R(1, 0));
    CHECK(Rt(0, 2) == R(2, 0));
    CHECK(Rt(1, 0) == R(0, 1));
    CHECK(Rt(1, 1) == R(1, 1));
    CHECK(Rt(1, 2) == R(2, 1));
    CHECK(Rt(2, 0) == R(0, 2));
    CHECK(Rt(2, 1) == R(1, 2));
    CHECK(Rt(2, 2) == R(2, 2));
}

TEST_CASE("ENU/NED conversion consistency") {
    // Converting a point through ENU and NED should give consistent results
    const double lat_rad = 0.5;  // ~28.6 degrees
    const double lon_rad = 0.25; // ~14.3 degrees

    const auto R_enu = earth::R_enu_from_ecf(lat_rad, lon_rad);
    const auto R_ned = earth::R_ned_from_ecf(lat_rad, lon_rad);

    // A point in ECEF
    dp::Point p_ecef{1000.0, 2000.0, 3000.0};

    // Convert to ENU and NED
    dp::Point p_enu = earth::mat_mul(R_enu, p_ecef);
    dp::Point p_ned = earth::mat_mul(R_ned, p_ecef);

    // ENU and NED should be related by: NED = (north, east, -up) = (enu.y, enu.x, -enu.z)
    CHECK(p_ned.x == doctest::Approx(p_enu.y).epsilon(1e-6)); // north = enu.y
    CHECK(p_ned.y == doctest::Approx(p_enu.x).epsilon(1e-6)); // east = enu.x
    CHECK(p_ned.z == doctest::Approx(-p_enu.z).epsilon(1e-6)); // down = -up
}
