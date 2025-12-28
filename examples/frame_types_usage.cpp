#include <concord/concord.hpp>
#include <iomanip>
#include <iostream>

using namespace concord;

int main() {
    std::cout << "=== Frame Types Usage Examples ===" << std::endl << std::endl;

    // ============================================================================
    // 1. ENU - East-North-Up (local tangent frame)
    // ============================================================================
    std::cout << "1. ENU (East-North-Up) Frame:" << std::endl;

    // Construction methods
    frame::ENU enu1;                           // Default constructor
    frame::ENU enu2{10.0, 20.0, 5.0};          // Direct construction (east, north, up)
    frame::ENU enu3{dp::Point{1.0, 2.0, 3.0}}; // From dp::Point

    std::cout << "   enu1 (default): " << enu1 << std::endl;
    std::cout << "   enu2 (10, 20, 5): " << enu2 << std::endl;
    std::cout << "   enu3 (from Point): " << enu3 << std::endl;

    // Semantic accessors
    std::cout << "   enu2.east() = " << enu2.east() << " m" << std::endl;
    std::cout << "   enu2.north() = " << enu2.north() << " m" << std::endl;
    std::cout << "   enu2.up() = " << enu2.up() << " m" << std::endl;

    // Generic accessors (x, y, z)
    std::cout << "   enu2.x() = " << enu2.x() << " (same as east)" << std::endl;
    std::cout << "   enu2.y() = " << enu2.y() << " (same as north)" << std::endl;
    std::cout << "   enu2.z() = " << enu2.z() << " (same as up)" << std::endl;

    // Mutable access
    enu1.east() = 100.0;
    enu1.north() = 200.0;
    enu1.up() = 50.0;
    std::cout << "   enu1 (modified): " << enu1 << std::endl;

    // Check if set
    std::cout << "   enu1.is_set() = " << (enu1.is_set() ? "true" : "false") << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 2. NED - North-East-Down (aerospace/navigation frame)
    // ============================================================================
    std::cout << "2. NED (North-East-Down) Frame:" << std::endl;

    frame::NED ned1;                             // Default constructor
    frame::NED ned2{100.0, 50.0, -10.0};         // Direct construction (north, east, down)
    frame::NED ned3{dp::Point{30.0, 40.0, 5.0}}; // From dp::Point

    std::cout << "   ned1 (default): " << ned1 << std::endl;
    std::cout << "   ned2 (100, 50, -10): " << ned2 << std::endl;
    std::cout << "   ned3 (from Point): " << ned3 << std::endl;

    // Semantic accessors
    std::cout << "   ned2.north() = " << ned2.north() << " m" << std::endl;
    std::cout << "   ned2.east() = " << ned2.east() << " m" << std::endl;
    std::cout << "   ned2.down() = " << ned2.down() << " m (negative = above ground)" << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 3. FRD - Forward-Right-Down (aerospace body frame)
    // ============================================================================
    std::cout << "3. FRD (Forward-Right-Down) Body Frame:" << std::endl;

    frame::FRD frd1;                // Default constructor
    frame::FRD frd2{5.0, 2.0, 0.5}; // Direct construction (forward, right, down)

    std::cout << "   frd1 (default): " << frd1 << std::endl;
    std::cout << "   frd2 (5, 2, 0.5): " << frd2 << std::endl;

    // Semantic accessors
    std::cout << "   frd2.forward() = " << frd2.forward() << " m" << std::endl;
    std::cout << "   frd2.right() = " << frd2.right() << " m" << std::endl;
    std::cout << "   frd2.down() = " << frd2.down() << " m" << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 4. FLU - Forward-Left-Up (ROS body frame convention)
    // ============================================================================
    std::cout << "4. FLU (Forward-Left-Up) Body Frame (ROS convention):" << std::endl;

    frame::FLU flu1;                // Default constructor
    frame::FLU flu2{5.0, 2.0, 0.5}; // Direct construction (forward, left, up)

    std::cout << "   flu1 (default): " << flu1 << std::endl;
    std::cout << "   flu2 (5, 2, 0.5): " << flu2 << std::endl;

    // Semantic accessors
    std::cout << "   flu2.forward() = " << flu2.forward() << " m" << std::endl;
    std::cout << "   flu2.left() = " << flu2.left() << " m" << std::endl;
    std::cout << "   flu2.up() = " << flu2.up() << " m" << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 5. Frame Tags and Traits
    // ============================================================================
    std::cout << "5. Frame Tags and Traits:" << std::endl;

    std::cout << "   ENU::name = \"" << frame::ENU::name << "\"" << std::endl;
    std::cout << "   NED::name = \"" << frame::NED::name << "\"" << std::endl;
    std::cout << "   FRD::name = \"" << frame::FRD::name << "\"" << std::endl;
    std::cout << "   FLU::name = \"" << frame::FLU::name << "\"" << std::endl;

    // Frame tags indicate the type of frame
    std::cout << "   ENU tag: LocalTangent" << std::endl;
    std::cout << "   NED tag: LocalTangent" << std::endl;
    std::cout << "   FRD tag: Body" << std::endl;
    std::cout << "   FLU tag: Body" << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 6. Practical Example: Robot Position
    // ============================================================================
    std::cout << "6. Practical Example - Robot Position:" << std::endl;

    // Robot position in ENU coordinates (relative to a local origin)
    frame::ENU robot_pos{15.5, 32.7, 0.0}; // 15.5m east, 32.7m north, ground level

    std::cout << "   Robot position (ENU): " << robot_pos << std::endl;
    std::cout << "   Distance from origin: "
              << std::sqrt(robot_pos.east() * robot_pos.east() + robot_pos.north() * robot_pos.north()) << " m"
              << std::endl;

    // Obstacle detected by sensor in body frame (FLU)
    frame::FLU obstacle_body{3.0, 0.5, 0.0}; // 3m forward, 0.5m left
    std::cout << "   Obstacle in body frame (FLU): " << obstacle_body << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 7. Practical Example: Drone Altitude
    // ============================================================================
    std::cout << "7. Practical Example - Drone Altitude:" << std::endl;

    // Drone position in NED (common in aerospace)
    frame::NED drone_ned{1000.0, 500.0, -100.0}; // 1km north, 500m east, 100m altitude

    std::cout << "   Drone position (NED): " << drone_ned << std::endl;
    std::cout << "   Altitude above ground: " << -drone_ned.down() << " m" << std::endl;
    std::cout << "   Horizontal distance: "
              << std::sqrt(drone_ned.north() * drone_ned.north() + drone_ned.east() * drone_ned.east()) << " m"
              << std::endl;

    return 0;
}
