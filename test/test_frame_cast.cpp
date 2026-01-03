#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord::frame;

// Reference origin for tests
const dp::Geo test_ref{48.8566, 2.3522, 35.0}; // Paris

TEST_CASE("frame_cast ENU <-> NED preserves origin") {
    const ENU enu{10.0, 20.0, 30.0, test_ref}; // east=10, north=20, up=30

    const NED ned = frame_cast<NED>(enu);
    CHECK(ned.north() == doctest::Approx(20.0));
    CHECK(ned.east() == doctest::Approx(10.0));
    CHECK(ned.down() == doctest::Approx(-30.0));
    // Origin should be preserved
    CHECK(ned.origin.latitude == test_ref.latitude);
    CHECK(ned.origin.longitude == test_ref.longitude);

    const ENU back = frame_cast<ENU>(ned);
    CHECK(back.east() == doctest::Approx(enu.east()));
    CHECK(back.north() == doctest::Approx(enu.north()));
    CHECK(back.up() == doctest::Approx(enu.up()));
    // Origin should still be preserved
    CHECK(back.origin.latitude == test_ref.latitude);
}

TEST_CASE("frame_cast FRD <-> FLU") {
    const FRD frd{1.0, 2.0, 3.0}; // forward=1, right=2, down=3

    const FLU flu = frame_cast<FLU>(frd);
    CHECK(flu.forward() == doctest::Approx(1.0));
    CHECK(flu.left() == doctest::Approx(-2.0));
    CHECK(flu.up() == doctest::Approx(-3.0));

    const FRD back = frame_cast<FRD>(flu);
    CHECK(back.forward() == doctest::Approx(frd.forward()));
    CHECK(back.right() == doctest::Approx(frd.right()));
    CHECK(back.down() == doctest::Approx(frd.down()));
}

TEST_CASE("frame_cast identity") {
    const ENU enu{1.0, 2.0, 3.0, test_ref};
    const ENU same = frame_cast<ENU>(enu);
    CHECK(same.east() == enu.east());
    CHECK(same.north() == enu.north());
    CHECK(same.up() == enu.up());
    CHECK(same.origin.latitude == enu.origin.latitude);
}

TEST_CASE("semantic accessors") {
    SUBCASE("ENU with origin") {
        ENU enu{10.0, 20.0, 30.0, test_ref};
        CHECK(enu.east() == 10.0);
        CHECK(enu.north() == 20.0);
        CHECK(enu.up() == 30.0);
        CHECK(enu.x() == 10.0);
        CHECK(enu.y() == 20.0);
        CHECK(enu.z() == 30.0);
        CHECK(enu.origin.latitude == test_ref.latitude);
    }

    SUBCASE("NED with origin") {
        NED ned{10.0, 20.0, 30.0, test_ref};
        CHECK(ned.north() == 10.0);
        CHECK(ned.east() == 20.0);
        CHECK(ned.down() == 30.0);
        CHECK(ned.origin.latitude == test_ref.latitude);
    }

    SUBCASE("FRD (body frame, no origin)") {
        FRD frd{1.0, 2.0, 3.0};
        CHECK(frd.forward() == 1.0);
        CHECK(frd.right() == 2.0);
        CHECK(frd.down() == 3.0);
    }

    SUBCASE("FLU (body frame, no origin)") {
        FLU flu{1.0, 2.0, 3.0};
        CHECK(flu.forward() == 1.0);
        CHECK(flu.left() == 2.0);
        CHECK(flu.up() == 3.0);
    }
}

TEST_CASE("frame tags and traits") {
    static_assert(ENU::tag == FrameTag::LocalTangent);
    static_assert(NED::tag == FrameTag::LocalTangent);
    static_assert(FRD::tag == FrameTag::Body);
    static_assert(FLU::tag == FrameTag::Body);

    static_assert(FrameTraits<ENU>::tag == FrameTag::LocalTangent);
    static_assert(FrameTraits<NED>::tag == FrameTag::LocalTangent);
    static_assert(FrameTraits<FRD>::tag == FrameTag::Body);
    static_assert(FrameTraits<FLU>::tag == FrameTag::Body);

    CHECK(true); // If we get here, static_asserts passed
}

TEST_CASE("ENU/NED extend dp::Loc") {
    // ENU IS-A dp::Loc
    ENU enu{100.0, 200.0, 50.0, test_ref};

    // Can use dp::Loc methods directly
    CHECK(enu.distance_from_origin() > 0);
    CHECK(enu.distance_from_origin_2d() == doctest::Approx(std::sqrt(100.0 * 100.0 + 200.0 * 200.0)));

    // Can check same origin
    ENU enu2{50.0, 60.0, 10.0, test_ref};
    CHECK(enu.same_origin(enu2));

    dp::Geo other_ref{52.5200, 13.4050, 34.0}; // Berlin
    ENU enu3{50.0, 60.0, 10.0, other_ref};
    CHECK_FALSE(enu.same_origin(enu3));
}

TEST_CASE("FRD/FLU extend dp::Point") {
    // FRD IS-A dp::Point
    FRD frd{3.0, 4.0, 0.0};

    // Can use dp::Point methods directly
    CHECK(frd.magnitude() == doctest::Approx(5.0));

    // FLU IS-A dp::Point
    FLU flu{3.0, 4.0, 0.0};
    CHECK(flu.magnitude() == doctest::Approx(5.0));
}
