#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("convert() builder errors without ref for WGS->ENU") {
    const earth::WGS wgs{48.8566, 2.3522, 35.0};
    const auto enu = convert(wgs).to<frame::ENU>().build();
    CHECK(enu.is_err());
    CHECK(enu.error().code == dp::Error::INVALID_ARGUMENT);
}

TEST_CASE("convert() builder WGS->ENU with ref") {
    const dp::Geo ref{48.8566, 2.3522, 35.0}; // Paris
    const earth::WGS wgs{48.8570, 2.3530, 40.0};

    const auto enu = convert(wgs).withRef(ref).to<frame::ENU>().build();
    REQUIRE(enu.is_ok());

    // ENU should carry the origin
    CHECK(enu.value().origin.latitude == ref.latitude);
    CHECK(enu.value().origin.longitude == ref.longitude);
}

TEST_CASE("convert() builder ENU->WGS without ref (self-contained)") {
    const dp::Geo ref{48.8566, 2.3522, 35.0};
    const earth::WGS original{48.8570, 2.3530, 40.0};

    // First convert to ENU
    const auto enu = convert(original).withRef(ref).to<frame::ENU>().build();
    REQUIRE(enu.is_ok());

    // Now convert back - NO REF NEEDED because ENU carries its origin!
    const auto back = convert(enu.value()).to<earth::WGS>().build();
    REQUIRE(back.is_ok());

    CHECK(back.value().latitude == doctest::Approx(original.latitude).epsilon(1e-10));
    CHECK(back.value().longitude == doctest::Approx(original.longitude).epsilon(1e-10));
    CHECK(back.value().altitude == doctest::Approx(original.altitude).epsilon(1e-5));
}

TEST_CASE("convert() builder chaining WGS->ENU->NED->WGS") {
    const dp::Geo ref{48.8566, 2.3522, 35.0}; // Paris
    const earth::WGS wgs{48.8570, 2.3530, 40.0};

    const auto roundtrip = convert(wgs).withRef(ref).to<frame::ENU>().to<frame::NED>().to<earth::WGS>().build();
    REQUIRE(roundtrip.is_ok());

    CHECK(roundtrip.value().latitude == doctest::Approx(wgs.latitude).epsilon(1e-10));
    CHECK(roundtrip.value().longitude == doctest::Approx(wgs.longitude).epsilon(1e-10));
    CHECK(roundtrip.value().altitude == doctest::Approx(wgs.altitude).epsilon(1e-5));
}

TEST_CASE("convert() builder withDatum alias works") {
    const dp::Geo ref{48.8566, 2.3522, 35.0};
    const earth::WGS wgs{48.8570, 2.3530, 40.0};

    // withDatum is an alias for withRef (backward compatibility)
    const auto enu = convert(wgs).withDatum(ref).to<frame::ENU>().build();
    REQUIRE(enu.is_ok());
    CHECK(enu.value().origin.latitude == ref.latitude);
}

TEST_CASE("convert() builder WGS<->ECF<->WGS") {
    const earth::WGS wgs{48.8566, 2.3522, 35.0};

    const auto roundtrip = convert(wgs).to<earth::ECF>().to<earth::WGS>().build();
    REQUIRE(roundtrip.is_ok());

    CHECK(roundtrip.value().latitude == doctest::Approx(wgs.latitude).epsilon(1e-10));
    CHECK(roundtrip.value().longitude == doctest::Approx(wgs.longitude).epsilon(1e-10));
    CHECK(roundtrip.value().altitude == doctest::Approx(wgs.altitude).epsilon(1e-6));
}
