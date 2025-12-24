#include <concord/concord.hpp>

#include <iostream>

int main() {
    using namespace concord;

    const earth::WGS paris{48.8566, 2.3522, 35.0};

    const auto utm = earth::to_utm(paris);
    if (!utm) {
        std::cerr << "to_utm failed: " << utm.error().message.c_str() << "\n";
        return 1;
    }

    std::cout << "UTM zone " << utm.value().zone << (utm.value().north ? "N" : "S") << ": " << utm.value().easting_m
              << ", " << utm.value().northing_m << "\n";
    return 0;
}

