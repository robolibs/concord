#include <concord/concord.hpp>
#include <doctest/doctest.h>

#include <cmath>

using namespace concord;

TEST_CASE("WGS -> ECF -> WGS round-trip") {
    const earth::WGS original{48.8566, 2.3522, 35.0}; // Paris

    const auto ecf = earth::to_ecf(original);
    const auto back = earth::to_wgs(ecf);

    CHECK(std::abs(back.latitude - original.latitude) < 1e-9);
    CHECK(std::abs(back.longitude - original.longitude) < 1e-9);
    CHECK(std::abs(back.altitude - original.altitude) < 1e-5);
}

TEST_CASE("ECF known values") {
    const earth::WGS wgs{0.0, 0.0, 0.0};
    const auto ecf = earth::to_ecf(wgs);

    CHECK(std::abs(ecf.p.x - earth::wgs84::a_m) < 1.0);
    CHECK(std::abs(ecf.p.y) < 1.0);
    CHECK(std::abs(ecf.p.z) < 1.0);
}
