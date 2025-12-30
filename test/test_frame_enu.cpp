#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("ENU construction with origin") {
    const dp::Geo ref{48.8566, 2.3522, 35.0}; // Paris
    const frame::ENU enu{100.0, 200.0, 50.0, ref};

    CHECK(enu.east() == 100.0);
    CHECK(enu.north() == 200.0);
    CHECK(enu.up() == 50.0);
    CHECK(enu.origin.latitude == ref.latitude);
    CHECK(enu.origin.longitude == ref.longitude);
    CHECK(enu.origin.altitude == ref.altitude);
}

TEST_CASE("ENU from dp::Loc") {
    const dp::Loc loc{dp::Point{10.0, 20.0, 5.0}, dp::Geo{52.0, 13.0, 100.0}};
    const frame::ENU enu{loc};

    CHECK(enu.east() == 10.0);
    CHECK(enu.north() == 20.0);
    CHECK(enu.up() == 5.0);
    CHECK(enu.origin.latitude == 52.0);
    CHECK(enu.origin.longitude == 13.0);
}

TEST_CASE("WGS <-> ENU round-trip near datum") {
    const dp::Geo ref{48.8566, 2.3522, 35.0}; // Paris
    const earth::WGS point{48.8570, 2.3530, 40.0};

    const frame::ENU enu = frame::to_enu(ref, point);

    // ENU now carries its origin
    CHECK(enu.origin.latitude == ref.latitude);
    CHECK(enu.origin.longitude == ref.longitude);

    // Convert back - no external datum needed!
    const earth::WGS back = frame::to_wgs(enu);

    CHECK(back.latitude == doctest::Approx(point.latitude).epsilon(1e-10));
    CHECK(back.longitude == doctest::Approx(point.longitude).epsilon(1e-10));
    CHECK(back.altitude == doctest::Approx(point.altitude).epsilon(1e-6));
}

TEST_CASE("ENU same_origin check") {
    const dp::Geo ref1{48.8566, 2.3522, 35.0};
    const dp::Geo ref2{52.5200, 13.4050, 34.0}; // Berlin

    const frame::ENU enu1{100.0, 200.0, 50.0, ref1};
    const frame::ENU enu2{150.0, 250.0, 60.0, ref1};
    const frame::ENU enu3{100.0, 200.0, 50.0, ref2};

    CHECK(enu1.same_origin(enu2));
    CHECK_FALSE(enu1.same_origin(enu3));
}

TEST_CASE("ENU distance calculations") {
    const dp::Geo ref{48.8566, 2.3522, 35.0};
    const frame::ENU enu1{0.0, 0.0, 0.0, ref};
    const frame::ENU enu2{3.0, 4.0, 0.0, ref};

    CHECK(enu1.distance_to(enu2) == doctest::Approx(5.0));
    CHECK(enu1.distance_to_2d(enu2) == doctest::Approx(5.0));
}
