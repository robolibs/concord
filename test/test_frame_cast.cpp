#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord::frame;

TEST_CASE("frame_cast ENU <-> NED") {
    const ENU enu{10.0, 20.0, 30.0}; // east=10, north=20, up=30

    const NED ned = frame_cast<NED>(enu);
    CHECK(ned.north() == doctest::Approx(20.0));
    CHECK(ned.east() == doctest::Approx(10.0));
    CHECK(ned.down() == doctest::Approx(-30.0));

    const ENU back = frame_cast<ENU>(ned);
    CHECK(back.east() == doctest::Approx(enu.east()));
    CHECK(back.north() == doctest::Approx(enu.north()));
    CHECK(back.up() == doctest::Approx(enu.up()));
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
    const ENU enu{1.0, 2.0, 3.0};
    const ENU same = frame_cast<ENU>(enu);
    CHECK(same.east() == enu.east());
    CHECK(same.north() == enu.north());
    CHECK(same.up() == enu.up());
}

TEST_CASE("semantic accessors") {
    SUBCASE("ENU") {
        ENU enu{10.0, 20.0, 30.0};
        CHECK(enu.east() == 10.0);
        CHECK(enu.north() == 20.0);
        CHECK(enu.up() == 30.0);
        CHECK(enu.x() == 10.0);
        CHECK(enu.y() == 20.0);
        CHECK(enu.z() == 30.0);
    }

    SUBCASE("NED") {
        NED ned{10.0, 20.0, 30.0};
        CHECK(ned.north() == 10.0);
        CHECK(ned.east() == 20.0);
        CHECK(ned.down() == 30.0);
    }

    SUBCASE("FRD") {
        FRD frd{1.0, 2.0, 3.0};
        CHECK(frd.forward() == 1.0);
        CHECK(frd.right() == 2.0);
        CHECK(frd.down() == 3.0);
    }

    SUBCASE("FLU") {
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
