#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("ENU <-> NED conversion") {
    const frame::ENU enu{dp::Point{1.0, 2.0, 3.0}};
    const frame::NED ned = frame::to_ned(enu);
    const frame::ENU back = frame::to_enu(ned);

    CHECK(back.p.x == doctest::Approx(enu.p.x));
    CHECK(back.p.y == doctest::Approx(enu.p.y));
    CHECK(back.p.z == doctest::Approx(enu.p.z));
}

TEST_CASE("WGS <-> NED round-trip near datum") {
    const frame::Datum datum{earth::WGS{48.8566, 2.3522, 35.0}}; // Paris
    const earth::WGS point{48.8570, 2.3530, 40.0};

    const frame::NED ned = frame::to_ned(datum, point);
    const earth::WGS back = frame::to_wgs(datum, ned);

    CHECK(back.lat_deg == doctest::Approx(point.lat_deg).epsilon(1e-10));
    CHECK(back.lon_deg == doctest::Approx(point.lon_deg).epsilon(1e-10));
    CHECK(back.alt_m == doctest::Approx(point.alt_m).epsilon(1e-6));
}

