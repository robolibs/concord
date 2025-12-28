#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("NED construction with origin") {
    const dp::Geo ref{48.8566, 2.3522, 35.0}; // Paris
    const frame::NED ned{100.0, 200.0, 50.0, ref};

    CHECK(ned.north() == 100.0);
    CHECK(ned.east() == 200.0);
    CHECK(ned.down() == 50.0);
    CHECK(ned.origin.latitude == ref.latitude);
    CHECK(ned.origin.longitude == ref.longitude);
}

TEST_CASE("ENU <-> NED conversion preserves origin") {
    const dp::Geo ref{48.8566, 2.3522, 35.0};
    const frame::ENU enu{1.0, 2.0, 3.0, ref}; // east=1, north=2, up=3

    const frame::NED ned = frame::to_ned(enu);

    // NED: north=2, east=1, down=-3
    CHECK(ned.north() == doctest::Approx(2.0));
    CHECK(ned.east() == doctest::Approx(1.0));
    CHECK(ned.down() == doctest::Approx(-3.0));

    // Origin should be preserved
    CHECK(ned.origin.latitude == ref.latitude);
    CHECK(ned.origin.longitude == ref.longitude);

    const frame::ENU back = frame::to_enu(ned);

    CHECK(back.east() == doctest::Approx(enu.east()));
    CHECK(back.north() == doctest::Approx(enu.north()));
    CHECK(back.up() == doctest::Approx(enu.up()));
    CHECK(back.origin.latitude == ref.latitude);
}

TEST_CASE("WGS <-> NED round-trip near datum") {
    const dp::Geo ref{48.8566, 2.3522, 35.0}; // Paris
    const earth::WGS point{48.8570, 2.3530, 40.0};

    const frame::NED ned = frame::to_ned(ref, point);

    // NED now carries its origin
    CHECK(ned.origin.latitude == ref.latitude);
    CHECK(ned.origin.longitude == ref.longitude);

    // Convert back - no external datum needed!
    const earth::WGS back = frame::to_wgs(ned);

    CHECK(back.latitude == doctest::Approx(point.latitude).epsilon(1e-10));
    CHECK(back.longitude == doctest::Approx(point.longitude).epsilon(1e-10));
    CHECK(back.altitude == doctest::Approx(point.altitude).epsilon(1e-6));
}

TEST_CASE("NED same_origin check") {
    const dp::Geo ref1{48.8566, 2.3522, 35.0};
    const dp::Geo ref2{52.5200, 13.4050, 34.0}; // Berlin

    const frame::NED ned1{100.0, 200.0, 50.0, ref1};
    const frame::NED ned2{150.0, 250.0, 60.0, ref1};
    const frame::NED ned3{100.0, 200.0, 50.0, ref2};

    CHECK(ned1.same_origin(ned2));
    CHECK_FALSE(ned1.same_origin(ned3));
}
