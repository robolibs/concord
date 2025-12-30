#include <cmath>
#include <concord/transform/temporal.hpp>
#include <doctest/doctest.h>

// ============================================================================
// Frame tags for testing
// ============================================================================
struct World {};
struct Odom {};
struct BaseLink {};
struct Camera {};

// ============================================================================
// Helper functions
// ============================================================================

// Create transform with translation only
template <typename To, typename From>
concord::frame::Transform<To, From> make_translation(double x, double y, double z) {
    return concord::frame::Transform<To, From>{concord::frame::Rotation<To, From>::identity(), dp::Point{x, y, z}};
}

// Create transform with Z rotation only
template <typename To, typename From> concord::frame::Transform<To, From> make_rotation_z(double angle_rad) {
    return concord::frame::Transform<To, From>{concord::frame::Rotation<To, From>::from_euler_zyx(angle_rad, 0, 0),
                                               dp::Point{0, 0, 0}};
}

// Check quaternion approximately equal
bool approx_equal_quat(const dp::Quaternion &a, const dp::Quaternion &b, double eps = 1e-10) {
    // Quaternions q and -q represent the same rotation
    bool same = std::abs(a.w - b.w) < eps && std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps &&
                std::abs(a.z - b.z) < eps;
    bool opposite = std::abs(a.w + b.w) < eps && std::abs(a.x + b.x) < eps && std::abs(a.y + b.y) < eps &&
                    std::abs(a.z + b.z) < eps;
    return same || opposite;
}

// Check point approximately equal
bool approx_equal_point(const dp::Point &a, const dp::Point &b, double eps = 1e-10) {
    return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps && std::abs(a.z - b.z) < eps;
}

// ============================================================================
// SLERP interpolation tests
// ============================================================================

TEST_CASE("slerp interpolation") {
    SUBCASE("identity to identity") {
        dp::Quaternion q1{1.0, 0.0, 0.0, 0.0};
        dp::Quaternion q2{1.0, 0.0, 0.0, 0.0};

        auto result = concord::transform::slerp(q1, q2, 0.5);
        CHECK(approx_equal_quat(result, dp::Quaternion{1.0, 0.0, 0.0, 0.0}));
    }

    SUBCASE("90 degree rotation") {
        // Identity quaternion
        dp::Quaternion q1{1.0, 0.0, 0.0, 0.0};
        // 90 degrees around Z axis: cos(45deg), 0, 0, sin(45deg)
        double half_angle = M_PI / 4.0;
        dp::Quaternion q2{std::cos(half_angle), 0.0, 0.0, std::sin(half_angle)};

        // t=0.5 should give 45 degree rotation
        auto result = concord::transform::slerp(q1, q2, 0.5);
        double expected_half = M_PI / 8.0;
        dp::Quaternion expected{std::cos(expected_half), 0.0, 0.0, std::sin(expected_half)};

        CHECK(approx_equal_quat(result, expected, 1e-6));
    }

    SUBCASE("180 degree rotation") {
        // Identity quaternion
        dp::Quaternion q1{1.0, 0.0, 0.0, 0.0};
        // 180 degrees around Z axis: cos(90deg), 0, 0, sin(90deg) = 0, 0, 0, 1
        dp::Quaternion q2{0.0, 0.0, 0.0, 1.0};

        // t=0.5 should give 90 degree rotation
        auto result = concord::transform::slerp(q1, q2, 0.5);
        double expected_half = M_PI / 4.0;
        dp::Quaternion expected{std::cos(expected_half), 0.0, 0.0, std::sin(expected_half)};

        CHECK(approx_equal_quat(result, expected, 1e-6));
    }

    SUBCASE("t=0 gives first quaternion") {
        dp::Quaternion q1{1.0, 0.0, 0.0, 0.0};
        double half_angle = M_PI / 6.0;
        dp::Quaternion q2{std::cos(half_angle), 0.0, 0.0, std::sin(half_angle)};

        auto result = concord::transform::slerp(q1, q2, 0.0);
        CHECK(approx_equal_quat(result, q1, 1e-10));
    }

    SUBCASE("t=1 gives second quaternion") {
        dp::Quaternion q1{1.0, 0.0, 0.0, 0.0};
        double half_angle = M_PI / 6.0;
        dp::Quaternion q2{std::cos(half_angle), 0.0, 0.0, std::sin(half_angle)};

        auto result = concord::transform::slerp(q1, q2, 1.0);
        CHECK(approx_equal_quat(result, q2, 1e-10));
    }
}

// ============================================================================
// lerp interpolation tests
// ============================================================================

TEST_CASE("lerp interpolation") {
    SUBCASE("basic linear interpolation") {
        dp::Point p1{0.0, 0.0, 0.0};
        dp::Point p2{10.0, 20.0, 30.0};

        auto result = concord::transform::lerp(p1, p2, 0.25);
        CHECK(result.x == doctest::Approx(2.5));
        CHECK(result.y == doctest::Approx(5.0));
        CHECK(result.z == doctest::Approx(7.5));
    }

    SUBCASE("t=0 gives first point") {
        dp::Point p1{1.0, 2.0, 3.0};
        dp::Point p2{10.0, 20.0, 30.0};

        auto result = concord::transform::lerp(p1, p2, 0.0);
        CHECK(approx_equal_point(result, p1));
    }

    SUBCASE("t=1 gives second point") {
        dp::Point p1{1.0, 2.0, 3.0};
        dp::Point p2{10.0, 20.0, 30.0};

        auto result = concord::transform::lerp(p1, p2, 1.0);
        CHECK(approx_equal_point(result, p2));
    }

    SUBCASE("t=0.5 gives midpoint") {
        dp::Point p1{0.0, 0.0, 0.0};
        dp::Point p2{10.0, 20.0, 30.0};

        auto result = concord::transform::lerp(p1, p2, 0.5);
        CHECK(result.x == doctest::Approx(5.0));
        CHECK(result.y == doctest::Approx(10.0));
        CHECK(result.z == doctest::Approx(15.0));
    }
}

// ============================================================================
// GenericTransform interpolation tests
// ============================================================================

TEST_CASE("GenericTransform interpolation") {
    using concord::transform::GenericTransform;
    using concord::transform::interpolate;

    SUBCASE("interpolate translation only") {
        GenericTransform tf1{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0}};
        GenericTransform tf2{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 20.0, 30.0}};

        auto result = interpolate(tf1, tf2, 0.5);
        CHECK(result.translation.x == doctest::Approx(5.0));
        CHECK(result.translation.y == doctest::Approx(10.0));
        CHECK(result.translation.z == doctest::Approx(15.0));
        CHECK(approx_equal_quat(result.rotation, dp::Quaternion{1.0, 0.0, 0.0, 0.0}));
    }

    SUBCASE("interpolate rotation only") {
        dp::Quaternion q1{1.0, 0.0, 0.0, 0.0};
        double half_angle = M_PI / 4.0; // 90 degrees total
        dp::Quaternion q2{std::cos(half_angle), 0.0, 0.0, std::sin(half_angle)};

        GenericTransform tf1{q1, dp::Point{0.0, 0.0, 0.0}};
        GenericTransform tf2{q2, dp::Point{0.0, 0.0, 0.0}};

        auto result = interpolate(tf1, tf2, 0.5);
        // Should be 45 degrees
        double expected_half = M_PI / 8.0;
        dp::Quaternion expected{std::cos(expected_half), 0.0, 0.0, std::sin(expected_half)};

        CHECK(approx_equal_quat(result.rotation, expected, 1e-6));
        CHECK(result.translation.x == doctest::Approx(0.0));
        CHECK(result.translation.y == doctest::Approx(0.0));
        CHECK(result.translation.z == doctest::Approx(0.0));
    }

    SUBCASE("interpolate both rotation and translation") {
        dp::Quaternion q1{1.0, 0.0, 0.0, 0.0};
        double half_angle = M_PI / 4.0;
        dp::Quaternion q2{std::cos(half_angle), 0.0, 0.0, std::sin(half_angle)};

        GenericTransform tf1{q1, dp::Point{0.0, 0.0, 0.0}};
        GenericTransform tf2{q2, dp::Point{10.0, 0.0, 0.0}};

        auto result = interpolate(tf1, tf2, 0.5);

        // Check translation is interpolated
        CHECK(result.translation.x == doctest::Approx(5.0));
        CHECK(result.translation.y == doctest::Approx(0.0));
        CHECK(result.translation.z == doctest::Approx(0.0));

        // Check rotation is interpolated
        double expected_half = M_PI / 8.0;
        dp::Quaternion expected_q{std::cos(expected_half), 0.0, 0.0, std::sin(expected_half)};
        CHECK(approx_equal_quat(result.rotation, expected_q, 1e-6));
    }
}

// ============================================================================
// TimedTransformBuffer basic operations tests
// ============================================================================

TEST_CASE("TimedTransformBuffer basic operations") {
    using concord::transform::GenericTransform;
    using concord::transform::TimedTransformBuffer;

    SUBCASE("empty buffer") {
        TimedTransformBuffer<> buffer;

        CHECK(buffer.empty());
        CHECK(buffer.size() == 0);
        CHECK_FALSE(buffer.latest().has_value());
        CHECK_FALSE(buffer.time_range().has_value());
    }

    SUBCASE("add single transform") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 2.0, 3.0}};
        buffer.add(1.0, tf); // 1 second

        CHECK_FALSE(buffer.empty());
        CHECK(buffer.size() == 1);
    }

    SUBCASE("add multiple transforms") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf1{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0}};
        GenericTransform tf2{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{2.0, 0.0, 0.0}};
        GenericTransform tf3{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{3.0, 0.0, 0.0}};

        buffer.add(1.0, tf1);
        buffer.add(2.0, tf2);
        buffer.add(3.0, tf3);

        CHECK(buffer.size() == 3);
    }

    SUBCASE("latest() returns most recent") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf1{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0}};
        GenericTransform tf2{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{2.0, 0.0, 0.0}};
        GenericTransform tf3{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{3.0, 0.0, 0.0}};

        buffer.add(1.0, tf1);
        buffer.add(3.0, tf3);
        buffer.add(2.0, tf2); // Add out of order

        auto latest = buffer.latest();
        REQUIRE(latest.has_value());
        CHECK(latest->translation.x == doctest::Approx(3.0));
    }

    SUBCASE("time_range() is correct") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0}};

        buffer.add(1.5, tf);
        buffer.add(3.5, tf);
        buffer.add(2.0, tf);

        auto range = buffer.time_range();
        REQUIRE(range.has_value());
        CHECK(range->first == doctest::Approx(1.5));
        CHECK(range->second == doctest::Approx(3.5));
    }
}

// ============================================================================
// TimedTransformBuffer interpolation tests
// ============================================================================

TEST_CASE("TimedTransformBuffer interpolation") {
    using concord::transform::GenericTransform;
    using concord::transform::TimedTransformBuffer;

    SUBCASE("exact timestamp match") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf1{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0}};
        GenericTransform tf2{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{2.0, 0.0, 0.0}};

        buffer.add(1.0, tf1);
        buffer.add(2.0, tf2);

        auto result = buffer.lookup(1.0);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(1.0));
    }

    SUBCASE("interpolate between two timestamps") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf1{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0}};
        GenericTransform tf2{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0}};

        buffer.add(1.0, tf1);
        buffer.add(2.0, tf2);

        // At t=1.5s, should be halfway
        auto result = buffer.lookup(1.5);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(5.0));
    }

    SUBCASE("timestamp before range returns nullopt") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0}};

        buffer.add(2.0, tf);
        buffer.add(3.0, tf);

        auto result = buffer.lookup(1.0);
        CHECK_FALSE(result.has_value());
    }

    SUBCASE("timestamp after range returns nullopt") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0}};

        buffer.add(1.0, tf);
        buffer.add(2.0, tf);

        auto result = buffer.lookup(3.0);
        CHECK_FALSE(result.has_value());
    }

    SUBCASE("single entry - exact match works") {
        TimedTransformBuffer<> buffer;

        GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0}};
        buffer.add(1.0, tf);

        // Exact match should work
        auto result_exact = buffer.lookup(1.0);
        REQUIRE(result_exact.has_value());
        CHECK(result_exact->translation.x == doctest::Approx(1.0));

        // Other timestamps should fail for single entry
        CHECK_FALSE(buffer.lookup(0.5).has_value());
        CHECK_FALSE(buffer.lookup(1.5).has_value());
    }
}

// ============================================================================
// TimedTransformBuffer buffer overflow tests
// ============================================================================

TEST_CASE("TimedTransformBuffer buffer overflow") {
    using concord::transform::GenericTransform;
    using concord::transform::TimedTransformBuffer;

    SUBCASE("buffer respects max size") {
        TimedTransformBuffer<5> buffer; // Max 5 entries

        GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0}};

        for (int i = 0; i < 10; ++i) {
            buffer.add(static_cast<double>(i), tf);
        }

        CHECK(buffer.size() <= 5);
    }

    SUBCASE("old entries are removed") {
        TimedTransformBuffer<3> buffer; // Max 3 entries

        GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0}};

        buffer.add(1.0, tf);
        buffer.add(2.0, tf);
        buffer.add(3.0, tf);
        buffer.add(4.0, tf); // Should evict t=1
        buffer.add(5.0, tf); // Should evict t=2

        auto range = buffer.time_range();
        REQUIRE(range.has_value());
        CHECK(range->first >= 3.0);
        CHECK(range->second == doctest::Approx(5.0));

        // Old timestamps should not be lookupable
        CHECK_FALSE(buffer.lookup(1.0).has_value());
        CHECK_FALSE(buffer.lookup(2.0).has_value());
    }
}

// ============================================================================
// TimedTransformTree frame registration tests
// ============================================================================

TEST_CASE("TimedTransformTree frame registration") {
    using concord::transform::TimedTransformTree;

    SUBCASE("register frames") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");
        tree.register_frame<BaseLink>("base_link");

        CHECK(tree.frame_count() == 3);
    }

    SUBCASE("has_frame") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");

        CHECK(tree.has_frame("world"));
        CHECK_FALSE(tree.has_frame("odom"));
    }

    SUBCASE("frame_names") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        auto names = tree.frame_names();
        CHECK(names.size() == 2);
        CHECK(std::find(names.begin(), names.end(), "world") != names.end());
        CHECK(std::find(names.begin(), names.end(), "odom") != names.end());
    }
}

// ============================================================================
// TimedTransformTree static transforms tests
// ============================================================================

TEST_CASE("TimedTransformTree static transforms") {
    using concord::transform::TimedTransformTree;

    SUBCASE("set and get static transform") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        auto tf = make_translation<World, Odom>(1.0, 2.0, 3.0);
        tree.set_static_transform("world", "odom", tf);

        // Should be available at any time
        auto result = tree.lookup("world", "odom", 100.0);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(1.0));
        CHECK(result->translation.y == doctest::Approx(2.0));
        CHECK(result->translation.z == doctest::Approx(3.0));
    }

    SUBCASE("static transform available at any time") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        auto tf = make_translation<World, Odom>(5.0, 0.0, 0.0);
        tree.set_static_transform("world", "odom", tf);

        // Should work at various times
        CHECK(tree.lookup("world", "odom", 0.0).has_value());
        CHECK(tree.lookup("world", "odom", 1000.0).has_value());
        CHECK(tree.lookup("world", "odom", -100.0).has_value());
    }
}

// ============================================================================
// TimedTransformTree dynamic transforms tests
// ============================================================================

TEST_CASE("TimedTransformTree dynamic transforms") {
    using concord::transform::TimedTransformTree;

    SUBCASE("set transform at specific time") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        auto tf = make_translation<World, Odom>(1.0, 0.0, 0.0);
        tree.set_transform("world", "odom", tf, 1.0);

        // Lookup should work at that time (with at least 2 entries for interpolation)
        tree.set_transform("world", "odom", tf, 2.0);

        CHECK(tree.has_transform("world", "odom"));
    }

    SUBCASE("lookup at exact time") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        auto tf1 = make_translation<World, Odom>(1.0, 0.0, 0.0);
        auto tf2 = make_translation<World, Odom>(2.0, 0.0, 0.0);

        tree.set_transform("world", "odom", tf1, 1.0);
        tree.set_transform("world", "odom", tf2, 2.0);

        auto result = tree.lookup("world", "odom", 1.0);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(1.0));
    }

    SUBCASE("lookup with interpolation") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        auto tf1 = make_translation<World, Odom>(0.0, 0.0, 0.0);
        auto tf2 = make_translation<World, Odom>(10.0, 0.0, 0.0);

        tree.set_transform("world", "odom", tf1, 1.0);
        tree.set_transform("world", "odom", tf2, 2.0);

        // At t=1.5s, should interpolate to x=5
        auto result = tree.lookup("world", "odom", 1.5);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(5.0));
    }
}

// ============================================================================
// TimedTransformTree path composition with time tests
// ============================================================================

TEST_CASE("TimedTransformTree path composition with time") {
    using concord::transform::TimedTransformTree;

    // World -> Odom -> BaseLink -> Camera
    // Each edge has different timestamps

    SUBCASE("compose path at common timestamp") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");
        tree.register_frame<BaseLink>("base_link");
        tree.register_frame<Camera>("camera");

        auto tf_wo = make_translation<World, Odom>(1.0, 0.0, 0.0);
        auto tf_ob = make_translation<Odom, BaseLink>(0.0, 2.0, 0.0);
        auto tf_bc = make_translation<BaseLink, Camera>(0.0, 0.0, 3.0);

        // All at t=1s and t=2s (need at least 2 entries for each edge)
        tree.set_transform("world", "odom", tf_wo, 1.0);
        tree.set_transform("world", "odom", tf_wo, 2.0);
        tree.set_transform("odom", "base_link", tf_ob, 1.0);
        tree.set_transform("odom", "base_link", tf_ob, 2.0);
        tree.set_transform("base_link", "camera", tf_bc, 1.0);
        tree.set_transform("base_link", "camera", tf_bc, 2.0);

        auto result = tree.lookup("world", "camera", 1.0);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(1.0));
        CHECK(result->translation.y == doctest::Approx(2.0));
        CHECK(result->translation.z == doctest::Approx(3.0));
    }

    SUBCASE("compose path with interpolation") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");
        tree.register_frame<BaseLink>("base_link");

        // World->Odom changes from x=0 to x=10 over t=[1s,2s]
        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(10.0, 0.0, 0.0), 2.0);

        // Odom->BaseLink is constant at y=5
        tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 5.0, 0.0), 1.0);
        tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 5.0, 0.0), 2.0);

        // At t=1.5s, world->odom should give x=5
        auto result = tree.lookup("world", "base_link", 1.5);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(5.0));
        CHECK(result->translation.y == doctest::Approx(5.0));
    }

    SUBCASE("mixed static and dynamic edges") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");
        tree.register_frame<BaseLink>("base_link");
        tree.register_frame<Camera>("camera");

        // World->Odom is dynamic
        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(10.0, 0.0, 0.0), 2.0);

        // Odom->BaseLink is dynamic
        tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 0.0, 0.0), 1.0);
        tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 10.0, 0.0), 2.0);

        // BaseLink->Camera is static
        tree.set_static_transform("base_link", "camera", make_translation<BaseLink, Camera>(0.0, 0.0, 1.0));

        // At t=1.5s, interpolate dynamic edges, use static for camera offset
        auto result = tree.lookup("world", "camera", 1.5);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(5.0));
        CHECK(result->translation.y == doctest::Approx(5.0));
        CHECK(result->translation.z == doctest::Approx(1.0));
    }
}

// ============================================================================
// TimedTransformTree time_range tests
// ============================================================================

TEST_CASE("TimedTransformTree time_range") {
    using concord::transform::TimedTransformTree;

    SUBCASE("single edge time range") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 5.0);

        auto range = tree.time_range("world", "odom");
        REQUIRE(range.has_value());
        CHECK(range->first == doctest::Approx(1.0));
        CHECK(range->second == doctest::Approx(5.0));
    }

    SUBCASE("multi-hop path time range (intersection)") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");
        tree.register_frame<BaseLink>("base_link");

        // World->Odom: t=[1s, 10s]
        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 10.0);

        // Odom->BaseLink: t=[3s, 7s]
        tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 0.0, 0.0), 3.0);
        tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 0.0, 0.0), 7.0);

        // Path time range should be intersection: [3s, 7s]
        auto range = tree.time_range("world", "base_link");
        REQUIRE(range.has_value());
        CHECK(range->first == doctest::Approx(3.0));
        CHECK(range->second == doctest::Approx(7.0));
    }
}

// ============================================================================
// TimedTransformTree can_transform with time tests
// ============================================================================

TEST_CASE("TimedTransformTree can_transform with time") {
    using concord::transform::TimedTransformTree;

    SUBCASE("within valid time range") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 5.0);

        CHECK(tree.can_transform("world", "odom", 1.0));
        CHECK(tree.can_transform("world", "odom", 3.0));
        CHECK(tree.can_transform("world", "odom", 5.0));
    }

    SUBCASE("outside time range") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 2.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 0.0), 4.0);

        CHECK_FALSE(tree.can_transform("world", "odom", 1.0));
        CHECK_FALSE(tree.can_transform("world", "odom", 5.0));
    }

    SUBCASE("static edge always valid") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        tree.set_static_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));

        CHECK(tree.can_transform("world", "odom", 0.0));
        CHECK(tree.can_transform("world", "odom", 1000.0));
        CHECK(tree.can_transform("world", "odom", -100.0));
    }
}

// ============================================================================
// TimedTransformTree lookup_latest tests
// ============================================================================

TEST_CASE("TimedTransformTree lookup_latest") {
    using concord::transform::TimedTransformTree;

    SUBCASE("returns latest transform") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(2.0, 0.0, 0.0), 2.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(3.0, 0.0, 0.0), 3.0);

        auto result = tree.lookup_latest("world", "odom");
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(3.0));
    }

    SUBCASE("path with mixed timestamps") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");
        tree.register_frame<BaseLink>("base_link");

        // World->Odom latest at t=5s
        tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(5.0, 0.0, 0.0), 5.0);

        // Odom->BaseLink latest at t=3s
        tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 1.0, 0.0), 1.0);
        tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 3.0, 0.0), 3.0);

        // lookup_latest should use the minimum of latest times (t=3)
        auto result = tree.lookup_latest("world", "base_link");
        REQUIRE(result.has_value());

        // At t=3, world->odom should be interpolated: (5-1)/(5-1) * (3-1) + 1 = 3
        // odom->base_link at t=3 is y=3
        CHECK(result->translation.x == doctest::Approx(3.0));
        CHECK(result->translation.y == doctest::Approx(3.0));
    }
}

// ============================================================================
// Additional edge case tests
// ============================================================================

TEST_CASE("TimedTransformTree edge cases") {
    using concord::transform::TimedTransformTree;

    SUBCASE("lookup non-existent frames") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");

        auto result = tree.lookup("world", "odom", 1.0);
        CHECK_FALSE(result.has_value());
    }

    SUBCASE("lookup disconnected frames") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");
        tree.register_frame<Camera>("camera");

        tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 2.0);
        // camera is not connected

        auto result = tree.lookup("world", "camera", 1.5);
        CHECK_FALSE(result.has_value());
    }

    SUBCASE("inverse transform with time") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        tree.set_transform("world", "odom", make_translation<World, Odom>(10.0, 0.0, 0.0), 1.0);
        tree.set_transform("world", "odom", make_translation<World, Odom>(10.0, 0.0, 0.0), 2.0);

        // Lookup in reverse direction
        auto result = tree.lookup("odom", "world", 1.5);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(-10.0));
    }

    SUBCASE("same frame lookup") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");

        auto result = tree.lookup("world", "world", 1.0);
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(0.0));
        CHECK(result->translation.y == doctest::Approx(0.0));
        CHECK(result->translation.z == doctest::Approx(0.0));
    }
}

TEST_CASE("TimedTransformTree rotation interpolation") {
    using concord::transform::TimedTransformTree;

    SUBCASE("rotation interpolation over time") {
        TimedTransformTree tree;

        tree.register_frame<World>("world");
        tree.register_frame<BaseLink>("base_link");

        // Rotate from 0 to 90 degrees around Z
        tree.set_transform("world", "base_link", make_rotation_z<World, BaseLink>(0.0), 0.0);
        tree.set_transform("world", "base_link", make_rotation_z<World, BaseLink>(M_PI / 2.0), 1.0);

        // At t=0.5s, should be 45 degrees
        auto result = tree.lookup("world", "base_link", 0.5);
        REQUIRE(result.has_value());

        // Apply rotation to test point (1, 0, 0)
        dp::Point p_in{1.0, 0.0, 0.0};
        dp::Point p_out = result->apply(p_in);

        // 45 degree rotation of (1, 0, 0) should give approximately (0.707, 0.707, 0)
        CHECK(p_out.x == doctest::Approx(std::cos(M_PI / 4.0)).epsilon(1e-6));
        CHECK(p_out.y == doctest::Approx(std::sin(M_PI / 4.0)).epsilon(1e-6));
        CHECK(p_out.z == doctest::Approx(0.0).epsilon(1e-6));
    }
}

TEST_CASE("TimedTransformTree clear") {
    using concord::transform::TimedTransformTree;

    TimedTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 1.0);
    tree.set_transform("world", "odom", make_translation<World, Odom>(2.0, 0.0, 0.0), 2.0);

    REQUIRE(tree.frame_count() == 2);

    tree.clear();

    CHECK(tree.frame_count() == 0);
    CHECK(tree.frame_names().empty());
    CHECK_FALSE(tree.has_frame("world"));
}

TEST_CASE("TimedTransformBuffer in_range") {
    using concord::transform::GenericTransform;
    using concord::transform::TimedTransformBuffer;

    TimedTransformBuffer<> buffer;

    GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0}};

    buffer.add(1.0, tf);
    buffer.add(5.0, tf);

    CHECK(buffer.in_range(1.0));
    CHECK(buffer.in_range(3.0));
    CHECK(buffer.in_range(5.0));
    CHECK_FALSE(buffer.in_range(0.5));
    CHECK_FALSE(buffer.in_range(6.0));
}

TEST_CASE("TimedTransformBuffer clear") {
    using concord::transform::GenericTransform;
    using concord::transform::TimedTransformBuffer;

    TimedTransformBuffer<> buffer;

    GenericTransform tf{dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0}};

    buffer.add(1.0, tf);
    buffer.add(2.0, tf);

    REQUIRE(buffer.size() == 2);

    buffer.clear();

    CHECK(buffer.empty());
    CHECK(buffer.size() == 0);
}
