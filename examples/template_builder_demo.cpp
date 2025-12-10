#include <concord/concord.hpp>
#include <iomanip>
#include <iostream>

using namespace concord;

int main() {
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "=== Basic Coordinate System Examples ===" << std::endl;

    // Define a reference datum (Seattle area)
    Datum seattle_datum{47.6062, -122.3321, 56.0};

    // Example 1: Point operations
    std::cout << "\n1. Point operations:" << std::endl;
    Point local_point(100.0, 200.0, 50.0);
    std::cout << "   Local Point: (" << local_point.x << ", " << local_point.y << ", " << local_point.z << ")"
              << std::endl;

    Point another_point(150.0, 250.0, 60.0);
    double dist = local_point.distance_to(another_point);
    std::cout << "   Distance to another point: " << dist << " meters" << std::endl;

    // Example 2: WGS coordinates
    std::cout << "\n2. WGS coordinates:" << std::endl;
    WGS portland(45.5152, -122.6784, 15.0);
    std::cout << "   Portland WGS: (" << portland.lat << ", " << portland.lon << ", " << portland.alt << ")"
              << std::endl;

    WGS seattle(47.6062, -122.3321, 56.0);
    double wgs_dist = seattle.distance_to(portland);
    std::cout << "   Distance Seattle to Portland: " << wgs_dist << " meters" << std::endl;

    double bearing = seattle.bearing_to(portland);
    std::cout << "   Bearing from Seattle to Portland: " << bearing << " degrees" << std::endl;

    // Example 3: ENU coordinates
    std::cout << "\n3. ENU coordinates:" << std::endl;
    ENU enu_point(100.0, 200.0, 50.0, seattle_datum);
    std::cout << "   ENU Point: (" << enu_point.x << ", " << enu_point.y << ", " << enu_point.z << ")" << std::endl;

    std::cout << "\nAll examples completed successfully!" << std::endl;

    return 0;
}
