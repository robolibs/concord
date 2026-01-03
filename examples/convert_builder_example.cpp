#include <concord/concord.hpp>

#include <iostream>

int main() {
    using namespace concord;

    // Define reference origin and a point to convert
    const dp::Geo ref{48.8566, 2.3522, 35.0}; // Paris (reference origin)
    const earth::WGS point{48.8570, 2.3530, 40.0};

    // Convert WGS to ENU using the fluent builder
    const auto enu = convert(point).withRef(ref).to<frame::ENU>().build();
    if (!enu) {
        std::cerr << "convert(WGS)->ENU failed: " << enu.error().message.c_str() << "\n";
        return 1;
    }

    std::cout << "ENU: east=" << enu.value().east() << ", north=" << enu.value().north() << ", up=" << enu.value().up()
              << "\n";
    std::cout << "Origin embedded: lat=" << enu.value().origin.latitude << ", lon=" << enu.value().origin.longitude
              << "\n";

    // Convert back to WGS - NO REF NEEDED because ENU carries its origin!
    const auto back = convert(enu.value()).to<earth::WGS>().build();
    if (!back) {
        std::cerr << "convert(ENU)->WGS failed: " << back.error().message.c_str() << "\n";
        return 1;
    }

    std::cout << "Back to WGS: lat=" << back.value().latitude << ", lon=" << back.value().longitude
              << ", alt=" << back.value().altitude << "\n";

    return 0;
}
