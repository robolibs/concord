#include <concord/concord.hpp>

#include <iostream>

int main() {
    using namespace concord;

    const earth::WGS origin{48.8566, 2.3522, 35.0}; // Paris
    const earth::WGS point{48.8570, 2.3530, 40.0};

    const frame::Datum datum{origin};
    const frame::ENU enu = frame::to_enu(datum, point);

    std::cout << "ENU: " << enu.p.x << ", " << enu.p.y << ", " << enu.p.z << "\n";
    return 0;
}

