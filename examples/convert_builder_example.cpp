#include <concord/concord.hpp>

#include <iostream>

int main() {
    using namespace concord;

    const frame::Datum datum{earth::WGS{48.8566, 2.3522, 35.0}}; // Paris
    const earth::WGS point{48.8570, 2.3530, 40.0};

    const auto enu = convert(point).withDatum(datum).to<frame::ENU>().build();
    if (!enu) {
        std::cerr << "convert(WGS)->ENU failed: " << enu.error().message.c_str() << "\n";
        return 1;
    }

    std::cout << "ENU: " << enu.value().p.x << ", " << enu.value().p.y << ", " << enu.value().p.z << "\n";

    const auto back = convert(enu.value()).withDatum(datum).to<earth::WGS>().build();
    if (!back) {
        std::cerr << "convert(ENU)->WGS failed: " << back.error().message.c_str() << "\n";
        return 1;
    }

    std::cout << "Back to WGS: " << back.value().latitude << ", " << back.value().longitude << ", "
              << back.value().altitude << "\n";

    return 0;
}
