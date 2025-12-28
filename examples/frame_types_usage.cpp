#include <concord/concord.hpp>
#include <iomanip>
#include <iostream>

using namespace concord;

int main() {
    std::cout << "=== Frame Types Usage Examples ===" << std::endl << std::endl;

    // Define a reference origin for local frames
    const dp::Geo paris{48.8566, 2.3522, 35.0}; // Paris as reference

    // ============================================================================
    // 1. ENU - East-North-Up (local tangent frame with origin)
    // ============================================================================
    std::cout << "1. ENU (East-North-Up) Frame:" << std::endl;
    std::cout << "   ENU now extends dp::Loc - carries both local coords AND origin!" << std::endl;

    // Construction methods
    frame::ENU enu1;                                           // Default constructor
    frame::ENU enu2{10.0, 20.0, 5.0, paris};                   // Direct construction (east, north, up, origin)
    frame::ENU enu3{dp::Point{1.0, 2.0, 3.0}, paris};          // From dp::Point + origin
    frame::ENU enu4{dp::Loc{dp::Point{5.0, 6.0, 7.0}, paris}}; // From dp::Loc

    std::cout << "   enu1 (default): " << enu1 << std::endl;
    std::cout << "   enu2 (10, 20, 5 @ Paris): " << enu2 << std::endl;
    std::cout << "   enu3 (from Point + origin): " << enu3 << std::endl;
    std::cout << "   enu4 (from dp::Loc): " << enu4 << std::endl;

    // Semantic accessors
    std::cout << "   enu2.east() = " << enu2.east() << " m" << std::endl;
    std::cout << "   enu2.north() = " << enu2.north() << " m" << std::endl;
    std::cout << "   enu2.up() = " << enu2.up() << " m" << std::endl;

    // Origin access (inherited from dp::Loc)
    std::cout << "   enu2.origin.latitude = " << enu2.origin.latitude << " deg" << std::endl;
    std::cout << "   enu2.origin.longitude = " << enu2.origin.longitude << " deg" << std::endl;

    // Generic accessors (x, y, z)
    std::cout << "   enu2.x() = " << enu2.x() << " (same as east)" << std::endl;
    std::cout << "   enu2.y() = " << enu2.y() << " (same as north)" << std::endl;
    std::cout << "   enu2.z() = " << enu2.z() << " (same as up)" << std::endl;

    // Mutable access
    enu1.east() = 100.0;
    enu1.north() = 200.0;
    enu1.up() = 50.0;
    enu1.origin = paris; // Set origin
    std::cout << "   enu1 (modified): " << enu1 << std::endl;

    // dp::Loc methods work directly on ENU
    std::cout << "   enu2.distance_from_origin() = " << enu2.distance_from_origin() << " m" << std::endl;
    std::cout << "   enu2.same_origin(enu3) = " << (enu2.same_origin(enu3) ? "true" : "false") << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 2. NED - North-East-Down (aerospace/navigation frame with origin)
    // ============================================================================
    std::cout << "2. NED (North-East-Down) Frame:" << std::endl;

    frame::NED ned1;                                    // Default constructor
    frame::NED ned2{100.0, 50.0, -10.0, paris};         // Direct construction (north, east, down, origin)
    frame::NED ned3{dp::Point{30.0, 40.0, 5.0}, paris}; // From dp::Point + origin

    std::cout << "   ned1 (default): " << ned1 << std::endl;
    std::cout << "   ned2 (100, 50, -10 @ Paris): " << ned2 << std::endl;
    std::cout << "   ned3 (from Point + origin): " << ned3 << std::endl;

    // Semantic accessors
    std::cout << "   ned2.north() = " << ned2.north() << " m" << std::endl;
    std::cout << "   ned2.east() = " << ned2.east() << " m" << std::endl;
    std::cout << "   ned2.down() = " << ned2.down() << " m (negative = above ground)" << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 3. FRD - Forward-Right-Down (aerospace body frame)
    // ============================================================================
    std::cout << "3. FRD (Forward-Right-Down) Body Frame:" << std::endl;
    std::cout << "   Body frames extend dp::Point - no global origin needed!" << std::endl;

    frame::FRD frd1;                // Default constructor
    frame::FRD frd2{5.0, 2.0, 0.5}; // Direct construction (forward, right, down)

    std::cout << "   frd1 (default): " << frd1 << std::endl;
    std::cout << "   frd2 (5, 2, 0.5): " << frd2 << std::endl;

    // Semantic accessors
    std::cout << "   frd2.forward() = " << frd2.forward() << " m" << std::endl;
    std::cout << "   frd2.right() = " << frd2.right() << " m" << std::endl;
    std::cout << "   frd2.down() = " << frd2.down() << " m" << std::endl;

    // dp::Point methods work directly on FRD
    std::cout << "   frd2.magnitude() = " << frd2.magnitude() << " m" << std::endl;
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
    std::cout << "   ENU tag: LocalTangent (extends dp::Loc)" << std::endl;
    std::cout << "   NED tag: LocalTangent (extends dp::Loc)" << std::endl;
    std::cout << "   FRD tag: Body (extends dp::Point)" << std::endl;
    std::cout << "   FLU tag: Body (extends dp::Point)" << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 6. Practical Example: Robot Position with Self-Contained Conversion
    // ============================================================================
    std::cout << "6. Practical Example - Robot Position:" << std::endl;

    // Robot position in ENU coordinates (relative to Paris origin)
    frame::ENU robot_pos{15.5, 32.7, 0.0, paris}; // 15.5m east, 32.7m north, ground level

    std::cout << "   Robot position (ENU): " << robot_pos << std::endl;
    std::cout << "   Distance from origin: " << robot_pos.distance_from_origin() << " m" << std::endl;

    // Convert back to WGS84 - NO EXTERNAL REFERENCE NEEDED!
    earth::WGS robot_wgs = frame::to_wgs(robot_pos);
    std::cout << "   Robot WGS84: " << robot_wgs << std::endl;

    // Obstacle detected by sensor in body frame (FLU)
    frame::FLU obstacle_body{3.0, 0.5, 0.0}; // 3m forward, 0.5m left
    std::cout << "   Obstacle in body frame (FLU): " << obstacle_body << std::endl;
    std::cout << std::endl;

    // ============================================================================
    // 7. Practical Example: Drone Altitude
    // ============================================================================
    std::cout << "7. Practical Example - Drone Altitude:" << std::endl;

    // Drone position in NED (common in aerospace)
    frame::NED drone_ned{1000.0, 500.0, -100.0, paris}; // 1km north, 500m east, 100m altitude

    std::cout << "   Drone position (NED): " << drone_ned << std::endl;
    std::cout << "   Altitude above ground: " << -drone_ned.down() << " m" << std::endl;
    std::cout << "   Horizontal distance: " << drone_ned.distance_from_origin_2d() << " m" << std::endl;

    // Convert to WGS84 - self-contained!
    earth::WGS drone_wgs = frame::to_wgs(drone_ned);
    std::cout << "   Drone WGS84: " << drone_wgs << std::endl;

    return 0;
}
