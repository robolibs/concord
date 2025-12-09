#include <concord/math.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("Vec3d operations") {
    SUBCASE("Vec3d creation and access") {
        Vec3d v{1.0, 2.0, 3.0};
        CHECK(v[0] == 1.0);
        CHECK(v[1] == 2.0);
        CHECK(v[2] == 3.0);
    }
}

TEST_CASE("Mat3d operations") {
    SUBCASE("Mat3d creation and access") {
        Mat3d m;
        m[0][0] = 1.0;
        m[0][1] = 2.0;
        m[0][2] = 3.0;
        m[1][0] = 4.0;
        m[1][1] = 5.0;
        m[1][2] = 6.0;
        m[2][0] = 7.0;
        m[2][1] = 8.0;
        m[2][2] = 9.0;
        CHECK(m[0][0] == 1.0);
        CHECK(m[1][1] == 5.0);
        CHECK(m[2][2] == 9.0);
    }

    SUBCASE("Mat3d vector multiplication") {
        Mat3d m;
        // Identity matrix
        m[0][0] = 1.0;
        m[0][1] = 0.0;
        m[0][2] = 0.0;
        m[1][0] = 0.0;
        m[1][1] = 1.0;
        m[1][2] = 0.0;
        m[2][0] = 0.0;
        m[2][1] = 0.0;
        m[2][2] = 1.0;

        Vec3d v{1.0, 2.0, 3.0};
        Vec3d result = m * v;
        CHECK(result[0] == doctest::Approx(1.0));
        CHECK(result[1] == doctest::Approx(2.0));
        CHECK(result[2] == doctest::Approx(3.0));
    }
}

TEST_CASE("Validation utilities") {
    SUBCASE("validate_finite") {
        CHECK_NOTHROW(validation::validate_finite(1.0, "test"));
        CHECK_THROWS_AS(validation::validate_finite(std::numeric_limits<double>::infinity(), "test"),
                        MathematicalException);
    }

    SUBCASE("validate_latitude") {
        CHECK_NOTHROW(validation::validate_latitude(45.0));
        CHECK_THROWS_AS(validation::validate_latitude(100.0), MathematicalException);
    }

    SUBCASE("validate_longitude") {
        CHECK_NOTHROW(validation::validate_longitude(90.0));
        CHECK_THROWS_AS(validation::validate_longitude(200.0), MathematicalException);
    }
}

TEST_CASE("Safe math utilities") {
    SUBCASE("safe_sqrt") {
        CHECK(safe_math::safe_sqrt(4.0) == doctest::Approx(2.0));
        CHECK(safe_math::safe_sqrt(-1.0) == doctest::Approx(0.0)); // Negative returns 0
    }

    SUBCASE("safe_asin") {
        CHECK(safe_math::safe_asin(0.5) == doctest::Approx(std::asin(0.5)));
        CHECK(safe_math::safe_asin(1.5) == doctest::Approx(std::asin(1.0))); // Clamped to 1.0
    }
}
