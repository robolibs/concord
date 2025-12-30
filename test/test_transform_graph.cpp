#include <cmath>
#include <concord/transform/tree.hpp>
#include <doctest/doctest.h>

// Define test frame tags
struct World {};
struct Odom {};
struct BaseLink {};
struct Camera {};
struct Lidar {};

// ============================================================================
// Helper functions
// ============================================================================

// Helper to create a translation-only transform
template <typename To, typename From>
concord::frame::Transform<To, From> make_translation(double x, double y, double z) {
    return concord::frame::Transform<To, From>{concord::frame::Rotation<To, From>::identity(), dp::Point{x, y, z}};
}

// Helper to create a rotation-only transform (around Z axis)
template <typename To, typename From> concord::frame::Transform<To, From> make_rotation_z(double angle_rad) {
    return concord::frame::Transform<To, From>{concord::frame::Rotation<To, From>::from_euler_zyx(angle_rad, 0, 0),
                                               dp::Point{0, 0, 0}};
}

// Helper to check if two points are approximately equal
bool approx_equal(const dp::Point &a, const dp::Point &b, double eps = 1e-10) {
    return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps && std::abs(a.z - b.z) < eps;
}

// ============================================================================
// Test cases
// ============================================================================

TEST_CASE("FrameGraph basic operations") {
    concord::transform::FrameGraph graph;

    SUBCASE("empty graph") {
        CHECK(graph.frame_count() == 0);
        CHECK(graph.transform_count() == 0);
        CHECK(graph.frame_names().empty());
    }

    SUBCASE("register single frame") {
        graph.register_frame<World>("world");
        CHECK(graph.frame_count() == 1);
        CHECK(graph.has_frame("world"));
        CHECK_FALSE(graph.has_frame("odom"));
    }

    SUBCASE("register multiple frames") {
        graph.register_frame<World>("world");
        graph.register_frame<Odom>("odom");
        graph.register_frame<BaseLink>("base_link");

        CHECK(graph.frame_count() == 3);
        CHECK(graph.has_frame("world"));
        CHECK(graph.has_frame("odom"));
        CHECK(graph.has_frame("base_link"));
    }

    SUBCASE("get_frame_id") {
        graph.register_frame<World>("world");
        graph.register_frame<Odom>("odom");

        auto world_id = graph.get_frame_id("world");
        auto odom_id = graph.get_frame_id("odom");

        REQUIRE(world_id.has_value());
        REQUIRE(odom_id.has_value());
        CHECK(world_id.value() != odom_id.value());
    }

    SUBCASE("get_frame_id for non-existent frame") {
        auto id = graph.get_frame_id("non_existent");
        CHECK_FALSE(id.has_value());
    }

    SUBCASE("frame_names returns all registered frames") {
        graph.register_frame<World>("world");
        graph.register_frame<Odom>("odom");
        graph.register_frame<BaseLink>("base_link");

        auto names = graph.frame_names();
        CHECK(names.size() == 3);
        CHECK(std::find(names.begin(), names.end(), "world") != names.end());
        CHECK(std::find(names.begin(), names.end(), "odom") != names.end());
        CHECK(std::find(names.begin(), names.end(), "base_link") != names.end());
    }

    SUBCASE("duplicate frame registration") {
        graph.register_frame<World>("world");
        // Second registration with same name should not increase count
        graph.register_frame<Odom>("world"); // Different type, same name
        CHECK(graph.frame_count() == 1);
    }
}

TEST_CASE("FrameGraph transform operations") {
    concord::transform::FrameGraph graph;

    graph.register_frame<World>("world");
    graph.register_frame<Odom>("odom");
    graph.register_frame<BaseLink>("base_link");

    SUBCASE("set and get transform") {
        auto tf = make_translation<World, Odom>(1.0, 2.0, 3.0);
        graph.set_transform("world", "odom", tf);

        CHECK(graph.has_transform("world", "odom"));
        CHECK(graph.transform_count() == 1);

        auto result = graph.get_transform("world", "odom");
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(1.0));
        CHECK(result->translation.y == doctest::Approx(2.0));
        CHECK(result->translation.z == doctest::Approx(3.0));
    }

    SUBCASE("multiple transforms") {
        auto tf1 = make_translation<World, Odom>(1.0, 0.0, 0.0);
        auto tf2 = make_translation<Odom, BaseLink>(0.0, 1.0, 0.0);

        graph.set_transform("world", "odom", tf1);
        graph.set_transform("odom", "base_link", tf2);

        CHECK(graph.transform_count() == 2);
        CHECK(graph.has_transform("world", "odom"));
        CHECK(graph.has_transform("odom", "base_link"));
    }

    SUBCASE("get non-existent transform") {
        auto result = graph.get_transform("world", "odom");
        CHECK_FALSE(result.has_value());
    }

    SUBCASE("update existing transform") {
        auto tf1 = make_translation<World, Odom>(1.0, 0.0, 0.0);
        auto tf2 = make_translation<World, Odom>(2.0, 0.0, 0.0);

        graph.set_transform("world", "odom", tf1);
        graph.set_transform("world", "odom", tf2);

        CHECK(graph.transform_count() == 1);
        auto result = graph.get_transform("world", "odom");
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(2.0));
    }
}

TEST_CASE("TransformTree path finding") {
    concord::transform::TransformTree tree;

    // Create a tree: World -> Odom -> BaseLink -> Camera
    //                                          -> Lidar
    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");
    tree.register_frame<Camera>("camera");
    tree.register_frame<Lidar>("lidar");

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 1.0, 0.0));
    tree.set_transform("base_link", "camera", make_translation<BaseLink, Camera>(0.5, 0.0, 0.5));
    tree.set_transform("base_link", "lidar", make_translation<BaseLink, Lidar>(0.0, 0.0, 1.0));

    SUBCASE("can_transform for direct edges") {
        CHECK(tree.can_transform("world", "odom"));
        CHECK(tree.can_transform("odom", "base_link"));
        CHECK(tree.can_transform("base_link", "camera"));
        CHECK(tree.can_transform("base_link", "lidar"));
    }

    SUBCASE("can_transform for inverse edges") {
        CHECK(tree.can_transform("odom", "world"));
        CHECK(tree.can_transform("base_link", "odom"));
        CHECK(tree.can_transform("camera", "base_link"));
    }

    SUBCASE("can_transform for multi-hop paths") {
        CHECK(tree.can_transform("world", "base_link"));
        CHECK(tree.can_transform("world", "camera"));
        CHECK(tree.can_transform("world", "lidar"));
        CHECK(tree.can_transform("camera", "lidar"));
    }

    SUBCASE("can_transform for reverse multi-hop paths") {
        CHECK(tree.can_transform("camera", "world"));
        CHECK(tree.can_transform("lidar", "world"));
        CHECK(tree.can_transform("lidar", "camera"));
    }

    SUBCASE("can_transform for non-existent frames") {
        CHECK_FALSE(tree.can_transform("world", "non_existent"));
        CHECK_FALSE(tree.can_transform("non_existent", "world"));
    }

    SUBCASE("can_transform same frame") { CHECK(tree.can_transform("world", "world")); }
}

TEST_CASE("TransformTree lookup - direct transforms") {
    concord::transform::TransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    auto tf_wo = make_translation<World, Odom>(10.0, 20.0, 30.0);
    tree.set_transform("world", "odom", tf_wo);

    SUBCASE("forward lookup") {
        auto result = tree.lookup("world", "odom");
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(10.0));
        CHECK(result->translation.y == doctest::Approx(20.0));
        CHECK(result->translation.z == doctest::Approx(30.0));
    }

    SUBCASE("inverse lookup") {
        auto result = tree.lookup("odom", "world");
        REQUIRE(result.has_value());
        // Inverse of pure translation is negative translation
        CHECK(result->translation.x == doctest::Approx(-10.0));
        CHECK(result->translation.y == doctest::Approx(-20.0));
        CHECK(result->translation.z == doctest::Approx(-30.0));
    }

    SUBCASE("same frame lookup") {
        auto result = tree.lookup("world", "world");
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(0.0));
        CHECK(result->translation.y == doctest::Approx(0.0));
        CHECK(result->translation.z == doctest::Approx(0.0));
    }
}

TEST_CASE("TransformTree lookup - chained transforms") {
    concord::transform::TransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");
    tree.register_frame<Camera>("camera");

    // Set World->Odom: translate by (1, 0, 0)
    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));
    // Set Odom->BaseLink: translate by (0, 2, 0)
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 2.0, 0.0));
    // Set BaseLink->Camera: translate by (0, 0, 3)
    tree.set_transform("base_link", "camera", make_translation<BaseLink, Camera>(0.0, 0.0, 3.0));

    SUBCASE("2-hop chain: world to base_link") {
        auto result = tree.lookup("world", "base_link");
        REQUIRE(result.has_value());
        // Composed: (1, 0, 0) + (0, 2, 0) = (1, 2, 0)
        CHECK(result->translation.x == doctest::Approx(1.0));
        CHECK(result->translation.y == doctest::Approx(2.0));
        CHECK(result->translation.z == doctest::Approx(0.0));
    }

    SUBCASE("3-hop chain: world to camera") {
        auto result = tree.lookup("world", "camera");
        REQUIRE(result.has_value());
        // Composed: (1, 0, 0) + (0, 2, 0) + (0, 0, 3) = (1, 2, 3)
        CHECK(result->translation.x == doctest::Approx(1.0));
        CHECK(result->translation.y == doctest::Approx(2.0));
        CHECK(result->translation.z == doctest::Approx(3.0));
    }

    SUBCASE("point transformation through chain") {
        auto result = tree.lookup("world", "camera");
        REQUIRE(result.has_value());

        dp::Point p_camera{0.0, 0.0, 0.0}; // Origin in camera frame
        dp::Point p_world = result->apply(p_camera);

        CHECK(p_world.x == doctest::Approx(1.0));
        CHECK(p_world.y == doctest::Approx(2.0));
        CHECK(p_world.z == doctest::Approx(3.0));
    }
}

TEST_CASE("TransformTree lookup - inverse chain") {
    concord::transform::TransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");

    tree.set_transform("world", "odom", make_translation<World, Odom>(5.0, 0.0, 0.0));
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 3.0, 0.0));

    SUBCASE("reverse single hop") {
        auto result = tree.lookup("odom", "world");
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(-5.0));
        CHECK(result->translation.y == doctest::Approx(0.0));
        CHECK(result->translation.z == doctest::Approx(0.0));
    }

    SUBCASE("reverse 2-hop chain") {
        auto result = tree.lookup("base_link", "world");
        REQUIRE(result.has_value());
        // Inverse composition: -(5, 0, 0) + -(0, 3, 0) = (-5, -3, 0)
        CHECK(result->translation.x == doctest::Approx(-5.0));
        CHECK(result->translation.y == doctest::Approx(-3.0));
        CHECK(result->translation.z == doctest::Approx(0.0));
    }

    SUBCASE("verify inverse by round-trip") {
        auto forward = tree.lookup("world", "base_link");
        auto backward = tree.lookup("base_link", "world");

        REQUIRE(forward.has_value());
        REQUIRE(backward.has_value());

        dp::Point p_original{7.0, 11.0, 13.0};
        dp::Point p_transformed = forward->apply(p_original);
        dp::Point p_back = backward->apply(p_transformed);

        CHECK(p_back.x == doctest::Approx(p_original.x).epsilon(1e-10));
        CHECK(p_back.y == doctest::Approx(p_original.y).epsilon(1e-10));
        CHECK(p_back.z == doctest::Approx(p_original.z).epsilon(1e-10));
    }
}

TEST_CASE("TransformTree transform composition accuracy") {
    concord::transform::TransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");

    const double angle = M_PI / 4.0; // 45 degrees

    // World->Odom: rotate 45 degrees around Z
    tree.set_transform("world", "odom", make_rotation_z<World, Odom>(angle));
    // Odom->BaseLink: translate (1, 0, 0) in odom frame
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(1.0, 0.0, 0.0));

    SUBCASE("rotation then translation") {
        auto result = tree.lookup("world", "base_link");
        REQUIRE(result.has_value());

        // A point at base_link origin should be at rotated position in world
        dp::Point p_base{0.0, 0.0, 0.0};
        dp::Point p_world = result->apply(p_base);

        // Translation (1, 0, 0) rotated by 45 deg = (cos45, sin45, 0) = (~0.707, ~0.707, 0)
        const double expected_x = std::cos(angle);
        const double expected_y = std::sin(angle);

        CHECK(p_world.x == doctest::Approx(expected_x).epsilon(1e-10));
        CHECK(p_world.y == doctest::Approx(expected_y).epsilon(1e-10));
        CHECK(p_world.z == doctest::Approx(0.0).epsilon(1e-10));
    }

    SUBCASE("manual composition vs lookup") {
        // Manually compose transforms
        auto rot = concord::frame::Rotation<World, Odom>::from_euler_zyx(angle, 0, 0);
        dp::Point trans{1.0, 0.0, 0.0};
        dp::Point rotated_trans = rot.apply(trans);

        auto lookup_result = tree.lookup("world", "base_link");
        REQUIRE(lookup_result.has_value());

        CHECK(lookup_result->translation.x == doctest::Approx(rotated_trans.x).epsilon(1e-10));
        CHECK(lookup_result->translation.y == doctest::Approx(rotated_trans.y).epsilon(1e-10));
        CHECK(lookup_result->translation.z == doctest::Approx(rotated_trans.z).epsilon(1e-10));
    }
}

TEST_CASE("TransformTree get_path") {
    concord::transform::TransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");
    tree.register_frame<Camera>("camera");

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 1.0, 0.0));
    tree.set_transform("base_link", "camera", make_translation<BaseLink, Camera>(0.0, 0.0, 1.0));

    SUBCASE("path from world to camera") {
        auto path = tree.get_path("world", "camera");
        REQUIRE(path.has_value());
        REQUIRE(path->size() == 4);
        CHECK((*path)[0] == "world");
        CHECK((*path)[1] == "odom");
        CHECK((*path)[2] == "base_link");
        CHECK((*path)[3] == "camera");
    }

    SUBCASE("path from camera to world (reverse)") {
        auto path = tree.get_path("camera", "world");
        REQUIRE(path.has_value());
        REQUIRE(path->size() == 4);
        CHECK((*path)[0] == "camera");
        CHECK((*path)[1] == "base_link");
        CHECK((*path)[2] == "odom");
        CHECK((*path)[3] == "world");
    }

    SUBCASE("path for same frame") {
        auto path = tree.get_path("world", "world");
        REQUIRE(path.has_value());
        REQUIRE(path->size() == 1);
        CHECK((*path)[0] == "world");
    }

    SUBCASE("path for non-existent frame") {
        auto path = tree.get_path("world", "non_existent");
        CHECK_FALSE(path.has_value());
    }

    SUBCASE("path for disconnected frames") {
        tree.register_frame<Lidar>("lidar"); // Not connected to the tree
        auto path = tree.get_path("world", "lidar");
        CHECK_FALSE(path.has_value());
    }
}

TEST_CASE("TransformTree clear") {
    concord::transform::TransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 1.0, 0.0));

    REQUIRE(tree.frame_count() == 3);
    REQUIRE(tree.transform_count() == 2);

    tree.clear();

    CHECK(tree.frame_count() == 0);
    CHECK(tree.transform_count() == 0);
    CHECK(tree.frame_names().empty());
    CHECK_FALSE(tree.has_frame("world"));
    CHECK_FALSE(tree.can_transform("world", "odom"));
}

TEST_CASE("TransformTree edge cases") {
    SUBCASE("lookup with non-existent source frame") {
        concord::transform::TransformTree tree;
        tree.register_frame<World>("world");

        auto result = tree.lookup("non_existent", "world");
        CHECK_FALSE(result.has_value());
    }

    SUBCASE("lookup with non-existent target frame") {
        concord::transform::TransformTree tree;
        tree.register_frame<World>("world");

        auto result = tree.lookup("world", "non_existent");
        CHECK_FALSE(result.has_value());
    }

    SUBCASE("single frame tree") {
        concord::transform::TransformTree tree;
        tree.register_frame<World>("world");

        CHECK(tree.frame_count() == 1);
        CHECK(tree.transform_count() == 0);
        CHECK(tree.can_transform("world", "world"));

        auto result = tree.lookup("world", "world");
        REQUIRE(result.has_value());
        CHECK(result->translation.x == doctest::Approx(0.0));
    }

    SUBCASE("two frames without transform") {
        concord::transform::TransformTree tree;
        tree.register_frame<World>("world");
        tree.register_frame<Odom>("odom");

        CHECK_FALSE(tree.can_transform("world", "odom"));
        CHECK_FALSE(tree.lookup("world", "odom").has_value());
    }
}

TEST_CASE("TransformTree with rotations") {
    concord::transform::TransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<BaseLink>("base_link");

    // 90 degree rotation around Z axis with translation
    auto rot = concord::frame::Rotation<World, BaseLink>::from_euler_zyx(M_PI / 2.0, 0, 0);
    concord::frame::Transform<World, BaseLink> tf{rot, dp::Point{10.0, 0.0, 0.0}};
    tree.set_transform("world", "base_link", tf);

    SUBCASE("forward transform with rotation") {
        auto result = tree.lookup("world", "base_link");
        REQUIRE(result.has_value());

        // Point (1, 0, 0) in base_link -> rotated 90 deg CCW then translated
        dp::Point p_base{1.0, 0.0, 0.0};
        dp::Point p_world = result->apply(p_base);

        // (1, 0, 0) rotated 90 deg CCW around Z = (0, 1, 0), then +10 in x
        CHECK(p_world.x == doctest::Approx(10.0).epsilon(1e-10));
        CHECK(p_world.y == doctest::Approx(1.0).epsilon(1e-10));
        CHECK(p_world.z == doctest::Approx(0.0).epsilon(1e-10));
    }

    SUBCASE("inverse transform with rotation") {
        auto result = tree.lookup("base_link", "world");
        REQUIRE(result.has_value());

        // Point (10, 0, 0) in world -> inverse transform to base_link
        // Inverse: first undo translation, then undo rotation
        dp::Point p_world{10.0, 0.0, 0.0};
        dp::Point p_base = result->apply(p_world);

        // (10, 0, 0) - (10, 0, 0) = (0, 0, 0), then rotate -90 deg = (0, 0, 0)
        CHECK(p_base.x == doctest::Approx(0.0).epsilon(1e-10));
        CHECK(p_base.y == doctest::Approx(0.0).epsilon(1e-10));
        CHECK(p_base.z == doctest::Approx(0.0).epsilon(1e-10));
    }
}

TEST_CASE("TransformTree branching structure") {
    concord::transform::TransformTree tree;

    // Create branching tree:
    //        World
    //          |
    //        Odom
    //          |
    //      BaseLink
    //       /    \
    //   Camera  Lidar

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");
    tree.register_frame<Camera>("camera");
    tree.register_frame<Lidar>("lidar");

    tree.set_transform("world", "odom", make_translation<World, Odom>(0.0, 0.0, 1.0));
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(1.0, 0.0, 0.0));
    tree.set_transform("base_link", "camera", make_translation<BaseLink, Camera>(0.5, 0.2, 0.3));
    tree.set_transform("base_link", "lidar", make_translation<BaseLink, Lidar>(-0.1, 0.0, 0.5));

    SUBCASE("camera to lidar through common ancestor") {
        auto result = tree.lookup("camera", "lidar");
        REQUIRE(result.has_value());

        // camera -> base_link -> lidar
        // camera to base_link: -0.5, -0.2, -0.3
        // base_link to lidar: -0.1, 0.0, 0.5
        // total: -0.6, -0.2, 0.2
        CHECK(result->translation.x == doctest::Approx(-0.6).epsilon(1e-10));
        CHECK(result->translation.y == doctest::Approx(-0.2).epsilon(1e-10));
        CHECK(result->translation.z == doctest::Approx(0.2).epsilon(1e-10));
    }

    SUBCASE("lidar to camera through common ancestor") {
        auto result = tree.lookup("lidar", "camera");
        REQUIRE(result.has_value());

        // lidar -> base_link -> camera
        // lidar to base_link: 0.1, 0.0, -0.5
        // base_link to camera: 0.5, 0.2, 0.3
        // total: 0.6, 0.2, -0.2
        CHECK(result->translation.x == doctest::Approx(0.6).epsilon(1e-10));
        CHECK(result->translation.y == doctest::Approx(0.2).epsilon(1e-10));
        CHECK(result->translation.z == doctest::Approx(-0.2).epsilon(1e-10));
    }

    SUBCASE("world to all sensors") {
        auto to_camera = tree.lookup("world", "camera");
        auto to_lidar = tree.lookup("world", "lidar");

        REQUIRE(to_camera.has_value());
        REQUIRE(to_lidar.has_value());

        // world->camera: (0,0,1) + (1,0,0) + (0.5,0.2,0.3) = (1.5, 0.2, 1.3)
        CHECK(to_camera->translation.x == doctest::Approx(1.5).epsilon(1e-10));
        CHECK(to_camera->translation.y == doctest::Approx(0.2).epsilon(1e-10));
        CHECK(to_camera->translation.z == doctest::Approx(1.3).epsilon(1e-10));

        // world->lidar: (0,0,1) + (1,0,0) + (-0.1,0,0.5) = (0.9, 0.0, 1.5)
        CHECK(to_lidar->translation.x == doctest::Approx(0.9).epsilon(1e-10));
        CHECK(to_lidar->translation.y == doctest::Approx(0.0).epsilon(1e-10));
        CHECK(to_lidar->translation.z == doctest::Approx(1.5).epsilon(1e-10));
    }
}
