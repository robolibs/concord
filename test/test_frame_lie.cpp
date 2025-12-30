#include <concord/concord.hpp>
#include <doctest/doctest.h>

#include <cmath>
#include <numbers>
#include <vector>

using namespace concord::frame;

// Define frame tags for testing
struct World {};
struct Body {};
struct Sensor {};

// ============================================================================
// Rotation exp/log tests
// ============================================================================

TEST_CASE("Rotation exp - identity from zero vector") {
    RotationTangent omega{0.0, 0.0, 0.0};
    auto rot = exp<World, Body>(omega);

    CHECK(is_identity(rot, 1e-10));
}

TEST_CASE("Rotation exp - 90 degree rotation around Z") {
    double angle = std::numbers::pi / 2.0;
    RotationTangent omega{0.0, 0.0, angle}; // rotation around Z axis

    auto rot = exp<World, Body>(omega);
    dp::Point p{1.0, 0.0, 0.0};
    auto result = rot.apply(p);

    CHECK(result.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(result.y == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Rotation log - identity gives zero vector") {
    auto rot = Rotation<World, Body>::identity();
    auto omega = log(rot);

    CHECK(omega[0] == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(omega[1] == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(omega[2] == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Rotation exp/log roundtrip") {
    double angle = 0.7;
    RotationTangent omega{0.3 * angle, 0.5 * angle, 0.8 * angle}; // arbitrary axis-angle

    auto rot = exp<World, Body>(omega);
    auto omega_back = log(rot);

    CHECK(omega_back[0] == doctest::Approx(omega[0]).epsilon(1e-10));
    CHECK(omega_back[1] == doctest::Approx(omega[1]).epsilon(1e-10));
    CHECK(omega_back[2] == doctest::Approx(omega[2]).epsilon(1e-10));
}

TEST_CASE("Rotation log/exp roundtrip") {
    auto rot = Rotation<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    auto omega = log(rot);
    auto rot_back = exp<World, Body>(omega);

    CHECK(is_approx(rot, rot_back, 1e-10));
}

// ============================================================================
// Transform exp/log tests
// ============================================================================

TEST_CASE("Transform exp - identity from zero twist") {
    TransformTangent twist{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    auto tf = exp<World, Body>(twist);

    CHECK(is_identity(tf, 1e-10));
}

TEST_CASE("Transform exp - pure translation") {
    TransformTangent twist{1.0, 2.0, 3.0, 0.0, 0.0, 0.0}; // translation only

    auto tf = exp<World, Body>(twist);
    dp::Point p{0.0, 0.0, 0.0};
    auto result = tf.apply(p);

    CHECK(result.x == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(result.y == doctest::Approx(2.0).epsilon(1e-10));
    CHECK(result.z == doctest::Approx(3.0).epsilon(1e-10));
}

TEST_CASE("Transform exp/log roundtrip") {
    TransformTangent twist{0.5, -0.3, 0.7, 0.1, 0.2, 0.15};

    auto tf = exp<World, Body>(twist);
    auto twist_back = log(tf);

    CHECK(twist_back[0] == doctest::Approx(twist[0]).epsilon(1e-8));
    CHECK(twist_back[1] == doctest::Approx(twist[1]).epsilon(1e-8));
    CHECK(twist_back[2] == doctest::Approx(twist[2]).epsilon(1e-8));
    CHECK(twist_back[3] == doctest::Approx(twist[3]).epsilon(1e-8));
    CHECK(twist_back[4] == doctest::Approx(twist[4]).epsilon(1e-8));
    CHECK(twist_back[5] == doctest::Approx(twist[5]).epsilon(1e-8));
}

// ============================================================================
// Interpolation tests
// ============================================================================

TEST_CASE("Rotation slerp - endpoints") {
    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);

    auto at_0 = slerp(r1, r2, 0.0);
    auto at_1 = slerp(r1, r2, 1.0);

    CHECK(is_approx(at_0, r1, 1e-10));
    CHECK(is_approx(at_1, r2, 1e-10));
}

TEST_CASE("Rotation slerp - midpoint") {
    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0); // 90 deg yaw

    auto mid = slerp(r1, r2, 0.5);

    // Midpoint should be 45 degree rotation
    dp::Point p{1.0, 0.0, 0.0};
    auto result = mid.apply(p);

    // 45 deg rotation of (1,0,0) -> (cos45, sin45, 0) = (0.707, 0.707, 0)
    CHECK(result.x == doctest::Approx(std::cos(std::numbers::pi / 4.0)).epsilon(1e-10));
    CHECK(result.y == doctest::Approx(std::sin(std::numbers::pi / 4.0)).epsilon(1e-10));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Rotation interpolate - same as slerp") {
    auto r1 = Rotation<World, Body>::from_euler_zyx(0.1, 0.2, 0.3);
    auto r2 = Rotation<World, Body>::from_euler_zyx(0.5, 0.6, 0.7);

    auto slerp_result = slerp(r1, r2, 0.3);
    auto interp_result = interpolate(r1, r2, 0.3);

    CHECK(is_approx(slerp_result, interp_result, 1e-10));
}

TEST_CASE("Transform interpolate - endpoints") {
    auto tf1 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0});
    auto tf2 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 20.0, 30.0});

    auto at_0 = interpolate(tf1, tf2, 0.0);
    auto at_1 = interpolate(tf1, tf2, 1.0);

    CHECK(is_approx(at_0, tf1, 1e-10));
    CHECK(is_approx(at_1, tf2, 1e-10));
}

TEST_CASE("Transform interpolate - midpoint translation") {
    auto tf1 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0});
    auto tf2 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0});

    auto mid = interpolate(tf1, tf2, 0.5);
    dp::Point origin{0.0, 0.0, 0.0};
    auto result = mid.apply(origin);

    CHECK(result.x == doctest::Approx(5.0).epsilon(1e-10));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-10));
}

// ============================================================================
// Averaging tests
// ============================================================================

TEST_CASE("Rotation average_two - midpoint") {
    auto r1 = Rotation<World, Body>::identity();
    auto r2 = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);

    auto avg = average_two(r1, r2);
    auto mid = slerp(r1, r2, 0.5);

    CHECK(is_approx(avg, mid, 1e-10));
}

TEST_CASE("Rotation average - single element") {
    auto r = Rotation<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    std::vector<Rotation<World, Body, double>> rotations = {r};

    auto result = average<World, Body, double>(std::span<const Rotation<World, Body, double>>(rotations));
    REQUIRE(result.has_value());
    CHECK(is_approx(*result, r, 1e-10));
}

TEST_CASE("Rotation average - identical elements") {
    auto r = Rotation<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    std::vector<Rotation<World, Body, double>> rotations = {r, r, r};

    auto result = average<World, Body, double>(std::span<const Rotation<World, Body, double>>(rotations));
    REQUIRE(result.has_value());
    CHECK(is_approx(*result, r, 1e-10));
}

TEST_CASE("Transform average_two") {
    auto tf1 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{0.0, 0.0, 0.0});
    auto tf2 = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{10.0, 0.0, 0.0});

    auto avg = average_two(tf1, tf2);
    auto mid = interpolate(tf1, tf2, 0.5);

    CHECK(is_approx(avg, mid, 1e-10));
}

// ============================================================================
// Utility function tests
// ============================================================================

TEST_CASE("Rotation angle - identity") {
    auto rot = Rotation<World, Body>::identity();
    CHECK(angle(rot) == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("Rotation angle - 90 degrees") {
    auto rot = Rotation<World, Body>::from_euler_zyx(std::numbers::pi / 2.0, 0.0, 0.0);
    CHECK(angle(rot) == doctest::Approx(std::numbers::pi / 2.0).epsilon(1e-10));
}

TEST_CASE("Rotation axis - Z axis rotation") {
    auto rot = Rotation<World, Body>::from_euler_zyx(0.5, 0.0, 0.0); // yaw only
    auto ax = axis(rot);

    CHECK(ax.x == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(ax.y == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(std::abs(ax.z) == doctest::Approx(1.0).epsilon(1e-10));
}

TEST_CASE("is_identity - rotation") {
    auto identity = Rotation<World, Body>::identity();
    auto non_identity = Rotation<World, Body>::from_euler_zyx(0.1, 0.0, 0.0);

    CHECK(is_identity(identity));
    CHECK_FALSE(is_identity(non_identity));
}

TEST_CASE("is_identity - transform") {
    auto identity = Transform<World, Body>::identity();
    auto non_identity = Transform<World, Body>::from_qt(dp::Quaternion{1.0, 0.0, 0.0, 0.0}, dp::Point{1.0, 0.0, 0.0});

    CHECK(is_identity(identity));
    CHECK_FALSE(is_identity(non_identity));
}

TEST_CASE("is_approx - rotation") {
    auto r1 = Rotation<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    auto r2 = Rotation<World, Body>::from_euler_zyx(0.5, 0.3, 0.1);
    auto r3 = Rotation<World, Body>::from_euler_zyx(0.6, 0.3, 0.1);

    CHECK(is_approx(r1, r2));
    CHECK_FALSE(is_approx(r1, r3));
}

// ============================================================================
// Hat/Vee operator tests
// ============================================================================

TEST_CASE("hat_so3/vee_so3 roundtrip") {
    RotationTangent omega{0.1, 0.2, 0.3};
    auto Omega = hat_so3(omega);
    auto omega_back = vee_so3(Omega);

    CHECK(omega_back[0] == doctest::Approx(omega[0]).epsilon(1e-10));
    CHECK(omega_back[1] == doctest::Approx(omega[1]).epsilon(1e-10));
    CHECK(omega_back[2] == doctest::Approx(omega[2]).epsilon(1e-10));
}

TEST_CASE("hat_se3/vee_se3 roundtrip") {
    TransformTangent twist{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    auto Omega = hat_se3(twist);
    auto twist_back = vee_se3(Omega);

    CHECK(twist_back[0] == doctest::Approx(twist[0]).epsilon(1e-10));
    CHECK(twist_back[1] == doctest::Approx(twist[1]).epsilon(1e-10));
    CHECK(twist_back[2] == doctest::Approx(twist[2]).epsilon(1e-10));
    CHECK(twist_back[3] == doctest::Approx(twist[3]).epsilon(1e-10));
    CHECK(twist_back[4] == doctest::Approx(twist[4]).epsilon(1e-10));
    CHECK(twist_back[5] == doctest::Approx(twist[5]).epsilon(1e-10));
}

// ============================================================================
// Jacobian tests
// ============================================================================

TEST_CASE("left_jacobian_so3 - identity at zero") {
    RotationTangent omega{0.0, 0.0, 0.0};
    auto J = left_jacobian_so3(omega);

    // At identity, left Jacobian should be identity matrix
    CHECK(J(0, 0) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(J(1, 1) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(J(2, 2) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(J(0, 1) == doctest::Approx(0.0).epsilon(1e-10));
    CHECK(J(0, 2) == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("left_jacobian_so3 inverse") {
    RotationTangent omega{0.1, 0.2, 0.3};
    auto J = left_jacobian_so3(omega);
    auto J_inv = left_jacobian_inverse_so3(omega);

    // J * J_inv should be identity
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += J(i, k) * J_inv(k, j);
            }
            double expected = (i == j) ? 1.0 : 0.0;
            CHECK(sum == doctest::Approx(expected).epsilon(1e-8));
        }
    }
}

// ============================================================================
// Adjoint tests
// ============================================================================

TEST_CASE("adjoint rotation - identity") {
    auto rot = Rotation<World, Body>::identity();
    auto adj = adjoint(rot);

    // Adjoint of identity rotation is identity matrix
    CHECK(adj(0, 0) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(adj(1, 1) == doctest::Approx(1.0).epsilon(1e-10));
    CHECK(adj(2, 2) == doctest::Approx(1.0).epsilon(1e-10));
}

TEST_CASE("adjoint transform - identity") {
    auto tf = Transform<World, Body>::identity();
    auto adj = adjoint(tf);

    // Adjoint of identity transform is 6x6 identity matrix
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            double expected = (i == j) ? 1.0 : 0.0;
            CHECK(adj(i, j) == doctest::Approx(expected).epsilon(1e-10));
        }
    }
}
