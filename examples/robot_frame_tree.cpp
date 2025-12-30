#include <cmath>
#include <concord/transform/tree.hpp>
#include <iomanip>
#include <iostream>

// ============================================================================
// Robot coordinate frame tags (empty structs for compile-time type safety)
// ============================================================================
struct World {};    // Global fixed frame
struct Odom {};     // Odometry frame (drifts over time)
struct BaseLink {}; // Robot base center
struct Camera {};   // RGB camera
struct Lidar {};    // 3D lidar
struct Imu {};      // IMU sensor

using namespace concord;

int main() {
    std::cout << "=== Robot Frame Tree Example ===" << std::endl << std::endl;

    // ========================================================================
    // 1. Create and populate transform tree
    // ========================================================================
    transform::TransformTree tree;

    // Register all frames
    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");
    tree.register_frame<Camera>("camera");
    tree.register_frame<Lidar>("lidar");
    tree.register_frame<Imu>("imu");

    // Print registered frames
    std::cout << "Registered frames: ";
    auto names = tree.frame_names();
    for (size_t i = 0; i < names.size(); ++i) {
        std::cout << names[i];
        if (i < names.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;

    // ========================================================================
    // 2. Set static sensor mounts (from URDF/calibration)
    // ========================================================================

    // Camera: 10cm forward, 20cm up from base, rotated 10 degrees down (pitch)
    auto camera_tf = frame::Transform<BaseLink, Camera>{
        frame::Rotation<BaseLink, Camera>::from_euler_zyx(0.0, -0.175, 0.0), // pitch down ~10 deg
        dp::Point{0.1, 0.0, 0.2}};
    tree.set_transform("base_link", "camera", camera_tf);

    // Lidar: centered, 30cm up
    tree.set_transform("base_link", "lidar",
                       frame::Transform<BaseLink, Lidar>::from_qt(dp::Quaternion{1, 0, 0, 0}, dp::Point{0, 0, 0.3}));

    // IMU: at base center (identity transform)
    tree.set_transform("base_link", "imu", frame::Transform<BaseLink, Imu>::identity());

    // ========================================================================
    // 3. Simulate robot motion
    // ========================================================================

    // Robot starts at origin, moves 1m forward in world
    tree.set_transform("world", "odom",
                       frame::Transform<World, Odom>::from_qt(dp::Quaternion{1, 0, 0, 0}, dp::Point{1.0, 0.0, 0.0}));

    // Odom to base_link (identity - no drift yet)
    tree.set_transform("odom", "base_link", frame::Transform<Odom, BaseLink>::identity());

    std::cout << "Total transforms: " << tree.transform_count() << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 4. Query sensor positions in world frame
    // ========================================================================
    std::cout << "--- Sensor Positions in World Frame ---" << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    auto camera_in_world = tree.lookup("world", "camera");
    if (camera_in_world) {
        dp::Point camera_pos = camera_in_world->apply(dp::Point{0, 0, 0});
        std::cout << "Camera: (" << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << ")" << std::endl;
    }

    auto lidar_in_world = tree.lookup("world", "lidar");
    if (lidar_in_world) {
        dp::Point lidar_pos = lidar_in_world->apply(dp::Point{0, 0, 0});
        std::cout << "Lidar:  (" << lidar_pos.x << ", " << lidar_pos.y << ", " << lidar_pos.z << ")" << std::endl;
    }

    auto imu_in_world = tree.lookup("world", "imu");
    if (imu_in_world) {
        dp::Point imu_pos = imu_in_world->apply(dp::Point{0, 0, 0});
        std::cout << "IMU:    (" << imu_pos.x << ", " << imu_pos.y << ", " << imu_pos.z << ")" << std::endl;
    }
    std::cout << std::endl;

    // ========================================================================
    // 5. Path finding
    // ========================================================================
    std::cout << "--- Path Finding ---" << std::endl;

    auto path = tree.get_path("world", "camera");
    if (path) {
        std::cout << "Path world -> camera: ";
        for (size_t i = 0; i < path->size(); ++i) {
            std::cout << (*path)[i];
            if (i < path->size() - 1) {
                std::cout << " -> ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // ========================================================================
    // 6. Point transformation
    // ========================================================================
    std::cout << "--- Point Transformation ---" << std::endl;

    dp::Point p_camera{1.0, 0.0, 0.0}; // 1m in front of camera
    if (camera_in_world) {
        dp::Point p_world = camera_in_world->apply(p_camera);
        std::cout << "Point (" << p_camera.x << ", " << p_camera.y << ", " << p_camera.z << ") in camera frame"
                  << std::endl;
        std::cout << "  -> (" << p_world.x << ", " << p_world.y << ", " << p_world.z << ") in world frame" << std::endl;
    }
    std::cout << std::endl;

    // ========================================================================
    // 7. Round-trip verification (world -> camera -> world)
    // ========================================================================
    std::cout << "--- Round-trip Verification ---" << std::endl;

    auto world_to_camera = tree.lookup("camera", "world");
    if (camera_in_world && world_to_camera) {
        dp::Point p_original{5.0, 3.0, 1.0};
        dp::Point p_camera_frame = world_to_camera->apply(p_original);
        dp::Point p_back = camera_in_world->apply(p_camera_frame);

        std::cout << "Original: (" << p_original.x << ", " << p_original.y << ", " << p_original.z << ")" << std::endl;
        std::cout << "After round-trip: (" << p_back.x << ", " << p_back.y << ", " << p_back.z << ")" << std::endl;

        // Calculate round-trip error
        double error = std::sqrt(std::pow(p_back.x - p_original.x, 2) + std::pow(p_back.y - p_original.y, 2) +
                                 std::pow(p_back.z - p_original.z, 2));
        std::cout << std::setprecision(6);
        std::cout << "Round-trip error: " << error << std::endl;
    }

    return 0;
}
