/**
 * @file datapod_interop_example.cpp
 * @brief Demonstrates interoperability between concord and datapod types
 *
 * This example shows how concord types (WGS, UTM, ENU, NED) work seamlessly
 * with datapod types (dp::Geo, dp::Utm, dp::Loc, dp::Point).
 *
 * Key concepts:
 * - concord::earth::WGS extends dp::Geo (same memory layout)
 * - concord::earth::UTM extends dp::Utm (same memory layout)
 * - concord::frame::ENU/NED extend dp::Loc (local coords + origin)
 * - concord::frame::FRD/FLU extend dp::Point (body-relative coords)
 */

#include <concord/concord.hpp>

#include <iostream>

int main() {
    using namespace concord;

    std::cout << "=== Datapod <-> Concord Interoperability ===\n\n";

    // =========================================================================
    // 1. dp::Geo <-> earth::WGS
    // =========================================================================
    std::cout << "1. dp::Geo <-> earth::WGS\n";
    std::cout << "   WGS inherits from dp::Geo - same memory layout!\n\n";

    // Create a dp::Geo (datapod's geodetic type)
    dp::Geo paris_geo{48.8566, 2.3522, 35.0};
    std::cout << "   dp::Geo paris: lat=" << paris_geo.latitude << ", lon=" << paris_geo.longitude
              << ", alt=" << paris_geo.altitude << "\n";

    // Convert to concord::earth::WGS (zero-copy, just reinterpret)
    earth::WGS paris_wgs{paris_geo};
    std::cout << "   earth::WGS paris: " << paris_wgs << "\n";

    // WGS adds concord-specific methods like lat_rad()/lon_rad()
    std::cout << "   Latitude in radians: " << paris_wgs.lat_rad() << "\n";

    // WGS IS-A dp::Geo, so it works anywhere dp::Geo is expected
    const dp::Geo &geo_ref = paris_wgs; // Implicit conversion
    std::cout << "   As dp::Geo ref: lat=" << geo_ref.latitude << "\n";

    // Use dp::Geo methods directly on WGS
    dp::Geo berlin_geo{52.5200, 13.4050, 34.0};
    double distance = paris_wgs.distance_to(berlin_geo); // dp::Geo method!
    std::cout << "   Distance Paris->Berlin: " << distance / 1000.0 << " km\n\n";

    // =========================================================================
    // 2. dp::Utm <-> earth::UTM
    // =========================================================================
    std::cout << "2. dp::Utm <-> earth::UTM\n";
    std::cout << "   UTM inherits from dp::Utm - same memory layout!\n\n";

    // Convert WGS to UTM using concord
    auto utm_result = earth::to_utm(paris_wgs);
    if (utm_result.is_ok()) {
        earth::UTM paris_utm = utm_result.value();
        std::cout << "   earth::UTM paris: " << paris_utm << "\n";

        // UTM IS-A dp::Utm
        const dp::Utm &utm_ref = paris_utm;
        std::cout << "   As dp::Utm ref: zone=" << utm_ref.zone << ", band=" << utm_ref.band << "\n";

        // Use dp::Utm methods directly
        std::cout << "   Central meridian: " << paris_utm.central_meridian() << " deg\n";
        std::cout << "   Is valid: " << (paris_utm.is_valid() ? "yes" : "no") << "\n\n";
    }

    // Create from dp::Utm directly
    dp::Utm raw_utm{31, 'U', 500000.0, 5400000.0, 100.0};
    earth::UTM concord_utm{raw_utm};
    std::cout << "   From dp::Utm: " << concord_utm << "\n\n";

    // =========================================================================
    // 3. dp::Loc <-> frame::ENU/NED (NOW EXTENDS dp::Loc!)
    // =========================================================================
    std::cout << "3. dp::Loc <-> frame::ENU/NED\n";
    std::cout << "   ENU/NED now EXTEND dp::Loc - they carry their origin!\n\n";

    // Create a dp::Loc (local position with reference origin)
    dp::Loc robot_loc{
        dp::Point{100.0, 50.0, 5.0},   // Local ENU coordinates (east, north, up)
        dp::Geo{48.8566, 2.3522, 35.0} // Origin (Paris)
    };
    std::cout << "   dp::Loc robot: local=(" << robot_loc.local.x << ", " << robot_loc.local.y << ", "
              << robot_loc.local.z << ")\n";
    std::cout << "   Origin: lat=" << robot_loc.origin.latitude << ", lon=" << robot_loc.origin.longitude << "\n";

    // Convert to ENU - ENU IS-A dp::Loc now!
    frame::ENU robot_enu{robot_loc};
    std::cout << "   As ENU: east=" << robot_enu.east() << ", north=" << robot_enu.north() << ", up=" << robot_enu.up()
              << "\n";
    std::cout << "   ENU carries origin: lat=" << robot_enu.origin.latitude << "\n";

    // Convert to NED (swaps axes: ENU -> NED)
    frame::NED robot_ned{robot_loc};
    std::cout << "   As NED: north=" << robot_ned.north() << ", east=" << robot_ned.east()
              << ", down=" << robot_ned.down() << "\n";
    std::cout << "   NED carries origin: lat=" << robot_ned.origin.latitude << "\n\n";

    // ENU/NED can now convert back to WGS without external reference!
    earth::WGS back_to_wgs = frame::to_wgs(robot_enu);
    std::cout << "   ENU -> WGS (self-contained): " << back_to_wgs << "\n\n";

    // =========================================================================
    // 4. dp::Point <-> frame::FRD/FLU (body frames)
    // =========================================================================
    std::cout << "4. dp::Point <-> frame::FRD/FLU\n";
    std::cout << "   Body frames extend dp::Point (no global origin needed)\n\n";

    dp::Point sensor_offset{0.5, 0.0, 0.3}; // 50cm forward, 30cm up

    // Create frame types from dp::Point - FLU IS-A dp::Point now!
    frame::FLU sensor_flu{sensor_offset};
    std::cout << "   FLU sensor: forward=" << sensor_flu.forward() << ", left=" << sensor_flu.left()
              << ", up=" << sensor_flu.up() << "\n";

    // Access as dp::Point (FLU inherits from dp::Point)
    const dp::Point &pt = sensor_flu.point();
    std::cout << "   As dp::Point: (" << pt.x << ", " << pt.y << ", " << pt.z << ")\n";

    // Use dp::Point methods directly on FLU
    std::cout << "   Magnitude: " << sensor_flu.magnitude() << " m\n\n";

    // =========================================================================
    // 5. Conversion pipeline with mixed types
    // =========================================================================
    std::cout << "5. Conversion pipeline with mixed types\n\n";

    // Start with dp::Geo
    dp::Geo start_geo{48.8570, 2.3530, 40.0};

    // Convert dp::Geo -> ENU using concord (ref is just dp::Geo now)
    auto enu_result = convert(earth::WGS{start_geo}).withRef(paris_geo).to<frame::ENU>().build();

    if (enu_result.is_ok()) {
        frame::ENU enu = enu_result.value();
        std::cout << "   dp::Geo -> ENU: " << enu << "\n";

        // ENU IS-A dp::Loc, so we can use dp::Loc methods
        std::cout << "   Distance from origin: " << enu.distance_from_origin() << " m\n";
        std::cout << "   Same origin check works: " << (enu.same_origin(robot_enu) ? "yes" : "no") << "\n";

        // Convert back to WGS - no external ref needed!
        earth::WGS back = frame::to_wgs(enu);
        std::cout << "   Back to WGS: " << back << "\n";
    }

    std::cout << "\n=== Done ===\n";
    return 0;
}
