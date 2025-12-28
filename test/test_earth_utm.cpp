#include <concord/concord.hpp>
#include <doctest/doctest.h>

#include <cmath>

using namespace concord;

TEST_CASE("WGS <-> UTM round-trip") {
    const earth::WGS paris{48.8566, 2.3522, 35.0};

    const auto utm = earth::to_utm(paris);
    REQUIRE(utm.is_ok());

    const auto back = earth::to_wgs(utm.value());
    REQUIRE(back.is_ok());

    CHECK(back.value().latitude == doctest::Approx(paris.latitude).epsilon(1e-9));
    CHECK(back.value().longitude == doctest::Approx(paris.longitude).epsilon(1e-9));
    CHECK(back.value().altitude == doctest::Approx(paris.altitude).epsilon(1e-6));
}

TEST_CASE("UTM latitude bounds error") {
    const earth::WGS wgs{85.0, 0.0, 0.0};
    const auto utm = earth::to_utm(wgs);
    CHECK(utm.is_err());
    CHECK(utm.error().code == dp::Error::OUT_OF_RANGE);
}
