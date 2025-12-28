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

    // ECF extends dp::Point, so x/y/z are direct members
    CHECK(std::abs(ecf.x - earth::wgs84::a_m) < 1.0);
    CHECK(std::abs(ecf.y) < 1.0);
    CHECK(std::abs(ecf.z) < 1.0);
}

TEST_CASE("ECF extends dp::Point") {
    const earth::ECF ecf{1000.0, 2000.0, 3000.0};

    // ECF IS-A dp::Point
    CHECK(ecf.x == 1000.0);
    CHECK(ecf.y == 2000.0);
    CHECK(ecf.z == 3000.0);

    // Can use dp::Point methods directly
    CHECK(ecf.magnitude() > 0);
    CHECK(ecf.is_set());

    // Access as dp::Point
    const dp::Point &pt = ecf.point();
    CHECK(pt.x == ecf.x);
}
