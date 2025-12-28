#include <concord/concord.hpp>

#include <iostream>

int main() {
    using namespace concord;

    // Define reference origin and a point to convert
    const dp::Geo ref{48.8566, 2.3522, 35.0}; // Paris (reference origin)
    const earth::WGS point{48.8570, 2.3530, 40.0};

    // Convert WGS to ENU - ENU now carries its origin!
    const frame::ENU enu = frame::to_enu(ref, point);

    std::cout << "ENU coordinates: east=" << enu.east() << ", north=" << enu.north() << ", up=" << enu.up() << "\n";
    std::cout << "Origin: lat=" << enu.origin.latitude << ", lon=" << enu.origin.longitude << "\n";

    // Convert back to WGS - no external reference needed!
    const earth::WGS back = frame::to_wgs(enu);
    std::cout << "Back to WGS: lat=" << back.latitude << ", lon=" << back.longitude << ", alt=" << back.altitude
              << "\n";

    return 0;
}
