#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("WGS <-> ENU round-trip near datum") {
    const frame::Datum datum{earth::WGS{48.8566, 2.3522, 35.0}}; // Paris
    const earth::WGS point{48.8570, 2.3530, 40.0};

    const frame::ENU enu = frame::to_enu(datum, point);
    const earth::WGS back = frame::to_wgs(datum, enu);

    CHECK(back.latitude == doctest::Approx(point.latitude).epsilon(1e-10));
    CHECK(back.longitude == doctest::Approx(point.longitude).epsilon(1e-10));
    CHECK(back.altitude == doctest::Approx(point.altitude).epsilon(1e-6));
}
