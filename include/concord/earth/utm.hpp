#pragma once

#include <datapod/datapod.hpp>

#include "types.hpp"
#include "wgs84.hpp"
#include <cmath>

namespace concord::earth {

    inline int utm_zone(double lon_deg) { return static_cast<int>(std::floor((lon_deg + 180.0) / 6.0)) + 1; }

    inline bool is_north(double lat_deg) { return lat_deg >= 0.0; }

    inline dp::Result<UTM> to_utm(const WGS &wgs) {
        if (wgs.lat_deg < -80.0 || wgs.lat_deg > 84.0) {
            return dp::Result<UTM>::err(dp::Error::out_of_range("Latitude out of UTM bounds (-80 to 84 deg)"));
        }

        const double lat_rad = wgs.lat_deg * wgs84::deg_to_rad;
        const double lon_rad = wgs.lon_deg * wgs84::deg_to_rad;

        const int zone = utm_zone(wgs.lon_deg);
        if (zone < 1 || zone > 60) {
            return dp::Result<UTM>::err(dp::Error::out_of_range("UTM zone out of range [1,60]"));
        }

        const double lon0 = ((zone - 1) * 6 - 180 + 3) * wgs84::deg_to_rad;

        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double tan_lat = std::tan(lat_rad);

        const double N = wgs84::a_m / std::sqrt(1.0 - wgs84::e2 * sin_lat * sin_lat);
        const double T = tan_lat * tan_lat;
        const double C = wgs84::ep2 * cos_lat * cos_lat;
        const double A = cos_lat * (lon_rad - lon0);

        const double M =
            wgs84::a_m *
            ((1.0 - wgs84::e2 / 4.0 - 3.0 * wgs84::e4 / 64.0 - 5.0 * wgs84::e6 / 256.0) * lat_rad -
             (3.0 * wgs84::e2 / 8.0 + 3.0 * wgs84::e4 / 32.0 + 45.0 * wgs84::e6 / 1024.0) * std::sin(2.0 * lat_rad) +
             (15.0 * wgs84::e4 / 256.0 + 45.0 * wgs84::e6 / 1024.0) * std::sin(4.0 * lat_rad) -
             (35.0 * wgs84::e6 / 3072.0) * std::sin(6.0 * lat_rad));

        const double A2 = A * A;
        const double A4 = A2 * A2;
        const double A6 = A4 * A2;

        double easting = wgs84::k0 * N *
                             (A + (1.0 - T + C) * A2 * A / 6.0 +
                              (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * wgs84::ep2) * A4 * A / 120.0) +
                         500000.0;

        double northing =
            wgs84::k0 * (M + N * tan_lat *
                                 (A2 / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0 +
                                  (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * wgs84::ep2) * A6 / 720.0));

        if (wgs.lat_deg < 0.0) {
            northing += 10000000.0;
        }

        return dp::Result<UTM>::ok(UTM{easting, northing, wgs.alt_m, zone, is_north(wgs.lat_deg)});
    }

    inline dp::Result<WGS> to_wgs(const UTM &utm) {
        if (utm.zone < 1 || utm.zone > 60) {
            return dp::Result<WGS>::err(dp::Error::out_of_range("UTM zone out of range [1,60]"));
        }

        double northing = utm.northing_m;
        if (!utm.north) {
            northing -= 10000000.0;
        }

        const double lon0 = ((utm.zone - 1) * 6 - 180 + 3) * wgs84::deg_to_rad;
        const double M = northing / wgs84::k0;
        const double mu = M / (wgs84::a_m * (1.0 - wgs84::e2 / 4.0 - 3.0 * wgs84::e4 / 64.0 - 5.0 * wgs84::e6 / 256.0));

        const double e1 = (1.0 - std::sqrt(1.0 - wgs84::e2)) / (1.0 + std::sqrt(1.0 - wgs84::e2));
        const double e1_2 = e1 * e1;
        const double e1_3 = e1_2 * e1;
        const double e1_4 = e1_3 * e1;

        const double phi1 = mu + (3.0 * e1 / 2.0 - 27.0 * e1_3 / 32.0) * std::sin(2.0 * mu) +
                            (21.0 * e1_2 / 16.0 - 55.0 * e1_4 / 32.0) * std::sin(4.0 * mu) +
                            (151.0 * e1_3 / 96.0) * std::sin(6.0 * mu) + (1097.0 * e1_4 / 512.0) * std::sin(8.0 * mu);

        const double sin_phi1 = std::sin(phi1);
        const double cos_phi1 = std::cos(phi1);
        const double tan_phi1 = std::tan(phi1);

        const double N1 = wgs84::a_m / std::sqrt(1.0 - wgs84::e2 * sin_phi1 * sin_phi1);
        const double T1 = tan_phi1 * tan_phi1;
        const double C1 = wgs84::ep2 * cos_phi1 * cos_phi1;
        const double R1 = wgs84::a_m * (1.0 - wgs84::e2) / std::pow(1.0 - wgs84::e2 * sin_phi1 * sin_phi1, 1.5);
        const double D = (utm.easting_m - 500000.0) / (N1 * wgs84::k0);

        const double D2 = D * D;
        const double D4 = D2 * D2;
        const double D6 = D4 * D2;

        const double lat =
            phi1 -
            (N1 * tan_phi1 / R1) *
                (D2 / 2.0 - (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * wgs84::ep2) * D4 / 24.0 +
                 (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * wgs84::ep2 - 3.0 * C1 * C1) * D6 / 720.0);

        const double lon =
            lon0 + (D - (1.0 + 2.0 * T1 + C1) * D2 * D / 6.0 +
                    (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * wgs84::ep2 + 24.0 * T1 * T1) * D4 * D / 120.0) /
                       cos_phi1;

        return dp::Result<WGS>::ok(WGS{lat * wgs84::rad_to_deg, lon * wgs84::rad_to_deg, utm.alt_m});
    }

} // namespace concord::earth
