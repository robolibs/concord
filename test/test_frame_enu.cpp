#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("WGS <-> ENU round-trip near datum") {
    const frame::Datum datum{earth::WGS{48.8566, 2.3522, 35.0}}; // Paris
    const earth::WGS point{48.8570, 2.3530, 40.0};

    const frame::ENU enu = frame::to_enu(datum, point);
    const earth::WGS back = frame::to_wgs(datum, enu);

    CHECK(back.lat_deg == doctest::Approx(point.lat_deg).epsilon(1e-10));
    CHECK(back.lon_deg == doctest::Approx(point.lon_deg).epsilon(1e-10));
    CHECK(back.alt_m == doctest::Approx(point.alt_m).epsilon(1e-6));
}

