#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("convert() builder errors without datum") {
    const earth::WGS wgs{48.8566, 2.3522, 35.0};
    const auto enu = convert(wgs).to<frame::ENU>().build();
    CHECK(enu.is_err());
    CHECK(enu.error().code == dp::Error::INVALID_ARGUMENT);
}

TEST_CASE("convert() builder chaining") {
    const frame::Datum datum{earth::WGS{48.8566, 2.3522, 35.0}}; // Paris
    const earth::WGS wgs{48.8570, 2.3530, 40.0};

    const auto roundtrip = convert(wgs).withDatum(datum).to<frame::ENU>().to<earth::WGS>().build();
    REQUIRE(roundtrip.is_ok());

    CHECK(roundtrip.value().latitude == doctest::Approx(wgs.latitude).epsilon(1e-12));
    CHECK(roundtrip.value().longitude == doctest::Approx(wgs.longitude).epsilon(1e-12));
    CHECK(roundtrip.value().altitude == doctest::Approx(wgs.altitude).epsilon(1e-5));
}
