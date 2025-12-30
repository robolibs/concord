#include <concord/concord.hpp>
#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

using namespace concord::frame;

// Define frame tags for testing
struct World {};
struct Body {};

// ============================================================================
// RotationSpline tests
// ============================================================================

TEST_CASE("RotationSpline - empty spline") {
    RotationSpline<World, Body> spline;

    CHECK(spline.size() == 0);
    CHECK_FALSE(spline.is_built());

    // Should return identity for empty spline
    auto rot = spline.evaluate(0.0);
    CHECK(is_identity(rot));
}

TEST_CASE("RotationSpline - single point") {
    RotationSpline<World, Body> spline;
    auto r1 = Rotation<World, Body>::from_euler_zyx(0.5, 0.0, 0.0);
    spline.add_point(r1);
    spline.build();

    CHECK(spline.size() == 1);
    CHECK_FALSE(spline.is_built()); // Need at least 2 points
}

TEST_CASE("RotationSpline - two points endpoints") {
    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);

    RotationSpline<World, Body> spline;
    spline.add_point(r1);
    spline.add_point(r2);
    spline.build();

    CHECK(spline.size() == 2);
    CHECK(spline.is_built());

    // At t=0, should be close to r1
    auto at_0 = spline.evaluate(0.0);
    CHECK(is_approx(at_0, r1, 1e-6));

    // At t=1, should be close to r2
    auto at_1 = spline.evaluate(1.0);
    CHECK(is_approx(at_1, r2, 1e-6));
}

TEST_CASE("RotationSpline - normalized evaluation") {
    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(0.5, 0.0, 0.0);
    auto r3 = Rotation<World, Body>::from_euler_zyx(1.0, 0.0, 0.0);

    RotationSpline<World, Body> spline;
    spline.add_point(r1);
    spline.add_point(r2);
    spline.add_point(r3);
    spline.build();

    // u=0 -> first point
    auto at_0 = spline.evaluate_normalized(0.0);
    CHECK(is_approx(at_0, r1, 1e-6));

    // u=1 -> last point
    auto at_1 = spline.evaluate_normalized(1.0);
    CHECK(is_approx(at_1, r3, 1e-6));
}

TEST_CASE("RotationSpline - smooth interpolation") {
    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 4.0, 0.0, 0.0);
    auto r3 = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);

    RotationSpline<World, Body> spline;
    spline.add_point(r1);
    spline.add_point(r2);
    spline.add_point(r3);
    spline.build();

    // Sample the spline and check it's smooth (angles increase monotonically)
    double prev_angle = 0.0;
    for (int i = 0; i <= 10; ++i) {
        double u = static_cast<double>(i) / 10.0;
        auto rot = spline.evaluate_normalized(u);
        double ang = angle(rot);

        CHECK(ang >= prev_angle - 1e-6); // Should be monotonically increasing
        prev_angle = ang;
    }
}

// ============================================================================
// TransformSpline tests
// ============================================================================

TEST_CASE("TransformSpline - empty spline") {
    TransformSpline<World, Body> spline;

    CHECK(spline.size() == 0);
    CHECK_FALSE(spline.is_built());

    auto tf = spline.evaluate(0.0);
    CHECK(is_identity(tf));
}

TEST_CASE("TransformSpline - two points endpoints") {
    auto tf1 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0});
    auto tf2 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0});

    TransformSpline<World, Body> spline;
    spline.add_point(tf1);
    spline.add_point(tf2);
    spline.build();

    CHECK(spline.is_built());

    // At t=0, should be at origin
    auto at_0 = spline.evaluate(0.0);
    CHECK(is_approx(at_0, tf1, 1e-6));

    // At t=1, should be at (10, 0, 0)
    auto at_1 = spline.evaluate(1.0);
    CHECK(is_approx(at_1, tf2, 1e-6));
}

TEST_CASE("TransformSpline - translation interpolation") {
    auto tf1 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0});
    auto tf2 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0});

    TransformSpline<World, Body> spline;
    spline.add_point(tf1);
    spline.add_point(tf2);
    spline.build();

    // Midpoint should be around (5, 0, 0)
    auto mid = spline.evaluate_normalized(0.5);
    dp::Point origin{0.0, 0.0, 0.0};
    auto result = mid.apply(origin);

    CHECK(result.x == doctest::Approx(5.0).epsilon(0.5)); // Allow some tolerance for spline
    CHECK(result.y == doctest::Approx(0.0).epsilon(0.1));
    CHECK(result.z == doctest::Approx(0.0).epsilon(0.1));
}

// ============================================================================
// TimedRotationSpline tests
// ============================================================================

TEST_CASE("TimedRotationSpline - empty") {
    TimedRotationSpline<World, Body> spline;

    CHECK(spline.size() == 0);
    CHECK_FALSE(spline.can_evaluate(0.0));
}

TEST_CASE("TimedRotationSpline - time range") {
    TimedRotationSpline<World, Body> spline;

    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(0.5, 0.0, 0.0);
    auto r3 = Rotation<World, Body>::from_euler_zyx(1.0, 0.0, 0.0);

    spline.add_point(1.0, r1);
    spline.add_point(2.0, r2);
    spline.add_point(3.0, r3);
    spline.build();

    auto [t_min, t_max] = spline.time_range();
    CHECK(t_min == doctest::Approx(1.0));
    CHECK(t_max == doctest::Approx(3.0));

    CHECK(spline.can_evaluate(1.5));
    CHECK(spline.can_evaluate(1.0));
    CHECK(spline.can_evaluate(3.0));
    CHECK_FALSE(spline.can_evaluate(0.5));
    CHECK_FALSE(spline.can_evaluate(3.5));
}

TEST_CASE("TimedRotationSpline - evaluate at endpoints") {
    TimedRotationSpline<World, Body> spline;

    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);

    spline.add_point(0.0, r1);
    spline.add_point(1.0, r2);
    spline.build();

    auto at_0 = spline.evaluate_at(0.0);
    CHECK(is_approx(at_0, r1, 1e-6));

    auto at_1 = spline.evaluate_at(1.0);
    CHECK(is_approx(at_1, r2, 1e-6));
}

TEST_CASE("TimedRotationSpline - out of order insertion") {
    TimedRotationSpline<World, Body> spline;

    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(0.5, 0.0, 0.0);
    auto r3 = Rotation<World, Body>::from_euler_zyx(1.0, 0.0, 0.0);

    // Add out of order - should be sorted internally
    spline.add_point(3.0, r3);
    spline.add_point(1.0, r1);
    spline.add_point(2.0, r2);
    spline.build();

    auto [t_min, t_max] = spline.time_range();
    CHECK(t_min == doctest::Approx(1.0));
    CHECK(t_max == doctest::Approx(3.0));
}

// ============================================================================
// TimedTransformSpline tests
// ============================================================================

TEST_CASE("TimedTransformSpline - evaluate at time") {
    TimedTransformSpline<World, Body> spline;

    auto tf1 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0});
    auto tf2 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0});

    spline.add_point(0.0, tf1);
    spline.add_point(1.0, tf2);
    spline.build();

    // At t=0.5, should be around (5, 0, 0)
    auto mid = spline.evaluate_at(0.5);
    dp::Point origin{0.0, 0.0, 0.0};
    auto result = mid.apply(origin);

    CHECK(result.x == doctest::Approx(5.0).epsilon(0.5));
}

TEST_CASE("TimedTransformSpline - clamping") {
    TimedTransformSpline<World, Body> spline;

    auto tf1 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0});
    auto tf2 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0});

    spline.add_point(1.0, tf1);
    spline.add_point(2.0, tf2);
    spline.build();

    // Before range - should clamp to first
    auto before = spline.evaluate_at(0.0);
    CHECK(is_approx(before, tf1, 1e-6));

    // After range - should clamp to last
    auto after = spline.evaluate_at(10.0);
    CHECK(is_approx(after, tf2, 1e-6));
}

// ============================================================================
// Sampling tests
// ============================================================================

TEST_CASE("sample_rotation_spline") {
    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);

    RotationSpline<World, Body> spline;
    spline.add_point(r1);
    spline.add_point(r2);
    spline.build();

    auto samples = sample_rotation_spline(spline, 5);
    CHECK(samples.size() == 5);

    // First sample should be close to r1
    CHECK(is_approx(samples[0], r1, 1e-6));

    // Last sample should be close to r2
    CHECK(is_approx(samples[4], r2, 1e-6));
}

TEST_CASE("sample_transform_spline") {
    auto tf1 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0});
    auto tf2 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0});

    TransformSpline<World, Body> spline;
    spline.add_point(tf1);
    spline.add_point(tf2);
    spline.build();

    auto samples = sample_transform_spline(spline, 5);
    CHECK(samples.size() == 5);

    CHECK(is_approx(samples[0], tf1, 1e-6));
    CHECK(is_approx(samples[4], tf2, 1e-6));
}

// ============================================================================
// Constructor tests
// ============================================================================

TEST_CASE("RotationSpline - construct from vector") {
    std::vector<Rotation<World, Body, double>> points = {Rotation<World, Body>::identity(),
                                                         Rotation<World, Body>::from_euler_zyx(0.5, 0.0, 0.0),
                                                         Rotation<World, Body>::from_euler_zyx(1.0, 0.0, 0.0)};

    RotationSpline<World, Body> spline(points);

    CHECK(spline.size() == 3);
    CHECK(spline.is_built());
}

TEST_CASE("TransformSpline - construct from vector") {
    std::vector<Transform<World, Body, double>> points = {
        Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0}),
        Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{5.0, 0.0, 0.0}),
        Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0})};

    TransformSpline<World, Body> spline(points);

    CHECK(spline.size() == 3);
    CHECK(spline.is_built());
}
