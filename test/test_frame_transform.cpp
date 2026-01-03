#include <concord/concord.hpp>
#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

using namespace concord::frame;

// Define some frame tags for testing
struct World {};
struct Body {};
struct Sensor {};

TEST_CASE("Rotation identity") {
    auto rot = Rotation<World, Body>::identity();
    dp::Point p{1.0, 2.0, 3.0};

    auto result = rot.apply(p);
    CHECK(result.x == doctest::Approx(1.0));
    CHECK(result.y == doctest::Approx(2.0));
    CHECK(result.z == doctest::Approx(3.0));
}

TEST_CASE("Rotation from Euler ZYX") {
    // 90 degree yaw rotation (around Z)
    auto rot = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);
    dp::Point p{1.0, 0.0, 0.0}; // pointing forward in body

    auto result = rot.apply(p);
    CHECK(result.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(result.y == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Rotation inverse") {
    auto rot = Rotation<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    dp::Point p{1.0, 2.0, 3.0};

    auto p_world = rot.apply(p);
    auto rot_inv = rot.inverse();
    auto p_back = rot_inv.apply(p_world);

    CHECK(p_back.x == doctest::Approx(p.x).epsilon(1e-10));
    CHECK(p_back.y == doctest::Approx(p.y).epsilon(1e-10));
    CHECK(p_back.z == doctest::Approx(p.z).epsilon(1e-10));
}

TEST_CASE("Rotation composition") {
    // R_WB: 90 deg yaw
    auto r_wb = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);
    // R_BS: 90 deg pitch
    auto r_bs = Rotation<Body, Sensor>::from_euler_zyx(0.0, std::numbers::pi / 2.0, 0.0);

    // Compose: R_WS = R_WB * R_BS
    auto r_ws = r_wb * r_bs;

    dp::Point p{1.0, 0.0, 0.0}; // forward in sensor
    auto result = r_ws.apply(p);

    // After 90 pitch then 90 yaw, (1,0,0) should go to approximately (0,1,0) then up
    // Actually: pitch 90 -> (0,0,-1), then yaw 90 -> (0,0,-1) (yaw doesn't affect z)
    CHECK(result.z == doctest::Approx(-1.0).epsilon(1e-10));
}

TEST_CASE("Transform identity") {
    auto tf = Transform<World, Body>::identity();
    dp::Point p{1.0, 2.0, 3.0};

    auto result = tf.apply(p);
    CHECK(result.x == doctest::Approx(1.0));
    CHECK(result.y == doctest::Approx(2.0));
    CHECK(result.z == doctest::Approx(3.0));
}

TEST_CASE("Transform translation only") {
    auto tf = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, // identity rotation
                                              dp::Point{10.0, 20.0, 30.0}         // translation
    );

    dp::Point p_body{1.0, 0.0, 0.0};
    dp::Point p_world = tf.apply(p_body);

    CHECK(p_world.x == doctest::Approx(11.0));
    CHECK(p_world.y == doctest::Approx(20.0));
    CHECK(p_world.z == doctest::Approx(30.0));
}

TEST_CASE("Transform inverse") {
    auto tf = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 20.0, 30.0});

    dp::Point p_body{1.0, 2.0, 3.0};
    dp::Point p_world = tf.apply(p_body);

    auto tf_inv = tf.inverse();
    dp::Point p_back = tf_inv.apply(p_world);

    CHECK(p_back.x == doctest::Approx(p_body.x).epsilon(1e-10));
    CHECK(p_back.y == doctest::Approx(p_body.y).epsilon(1e-10));
    CHECK(p_back.z == doctest::Approx(p_body.z).epsilon(1e-10));
}

TEST_CASE("Transform composition") {
    // T_world_body: body is at (10,0,0) in world, no rotation
    auto t_wb = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0});

    // T_body_sensor: sensor is at (0,1,0) in body, no rotation
    auto t_bs = Transform<Body, Sensor>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 1.0, 0.0});

    // Compose: T_world_sensor = T_world_body * T_body_sensor
    auto t_ws = t_wb * t_bs;

    dp::Point p_sensor{0.0, 0.0, 0.0}; // origin of sensor frame
    dp::Point p_world = t_ws.apply(p_sensor);

    CHECK(p_world.x == doctest::Approx(10.0));
    CHECK(p_world.y == doctest::Approx(1.0));
    CHECK(p_world.z == doctest::Approx(0.0));
}

TEST_CASE("Transform operator*") {
    auto tf = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{5.0, 5.0, 5.0});

    dp::Point p{1.0, 1.0, 1.0};
    dp::Point result = tf * p;

    CHECK(result.x == doctest::Approx(6.0));
    CHECK(result.y == doctest::Approx(6.0));
    CHECK(result.z == doctest::Approx(6.0));
}
