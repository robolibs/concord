#include <concord/concord.hpp>
#include <doctest/doctest.h>
#include <cmath>

using namespace concord;

TEST_CASE("Template-based coordinate builder pattern") {
    
    SUBCASE("Point to ENU to WGS conversion chain") {
        // Setup
        Datum seattle_datum(47.6062, -122.3321, 56.0);
        Point local_point(100.0, 200.0, 50.0);
        
        // Fluent conversion chain
        auto wgs_result = convert(local_point)
            .withDatum(seattle_datum)
            .as<ENU>()
            .to<WGS>()
            .build();
        
        // Verify result is reasonable
        CHECK(wgs_result.lat > 47.0);
        CHECK(wgs_result.lat < 48.0);
        CHECK(wgs_result.lon > -123.0);
        CHECK(wgs_result.lon < -122.0);
        CHECK(wgs_result.alt > 100.0);
    }
    
    SUBCASE("WGS to ENU conversion") {
        // Setup
        Datum origin_datum(47.6062, -122.3321, 56.0);
        WGS portland(45.5152, -122.6784, 15.0);
        
        // Convert WGS to ENU
        auto enu_result = convert(portland)
            .withDatum(origin_datum)
            .as<ENU>()
            .get();
        
        // Verify ENU coordinates are reasonable
        CHECK(enu_result.x != 0.0);  // Should have some offset
        CHECK(enu_result.y != 0.0);  // Should have some offset
        CHECK(enu_result.z < 0.0);   // Portland is lower than Seattle, so Z should be negative
        CHECK(std::abs(enu_result.z) > 1000.0);  // Should be significant distance due to geographic separation
    }
    
    SUBCASE("Direct type conversion without chaining") {
        // Setup
        Datum datum(47.6062, -122.3321, 56.0);
        Point point(10.0, 20.0, 5.0);
        
        // Direct conversion
        auto enu_direct = convert(point)
            .withDatum(datum)
            .as<ENU>()
            .get();
        
        // Verify basic properties
        CHECK(enu_direct.x == 10.0);
        CHECK(enu_direct.y == 20.0);
        CHECK(enu_direct.z == 5.0);
        CHECK(enu_direct.datum.lat == datum.lat);
        CHECK(enu_direct.datum.lon == datum.lon);
    }
    
    SUBCASE("Multiple coordinate system support") {
        // Setup
        Datum datum(40.7128, -74.0060, 10.0);  // New York
        Point nyc_point(500.0, 300.0, 100.0);
        
        // Test Point -> ENU -> WGS -> ENU round trip
        auto wgs_intermediate = convert(nyc_point)
            .withDatum(datum)
            .as<ENU>()
            .to<WGS>()
            .get();
        
        auto enu_final = convert(wgs_intermediate)
            .withDatum(datum)
            .as<ENU>()
            .get();
        
        // Round trip should preserve original coordinates (within tolerance)
        CHECK(std::abs(enu_final.x - nyc_point.x) < 0.01);
        CHECK(std::abs(enu_final.y - nyc_point.y) < 0.01);
        CHECK(std::abs(enu_final.z - nyc_point.z) < 0.01);
    }
}

TEST_CASE("Coordinate system validation and properties") {
    
    SUBCASE("WGS84 coordinate validation") {
        // Valid coordinates - these should construct successfully
        WGS valid1(47.6062, -122.3321, 56.0);
        WGS valid2(0.0, 0.0, 0.0);
        WGS valid3(90.0, 180.0, 1000.0);
        
        CHECK(valid1.lat == 47.6062);
        CHECK(valid2.lat == 0.0);
        CHECK(valid3.lat == 90.0);
        
        // Invalid coordinates should throw
        CHECK_THROWS(WGS(91.0, 0.0, 0.0));    // Latitude > 90
        CHECK_THROWS(WGS(-91.0, 0.0, 0.0));   // Latitude < -90  
        CHECK_THROWS(WGS(0.0, 181.0, 0.0));   // Longitude > 180
        CHECK_THROWS(WGS(0.0, -181.0, 0.0));  // Longitude < -180
    }
    
    SUBCASE("ENU coordinate properties") {
        Datum origin(37.4220, -122.0841, 0.0);  // Stanford
        ENU enu_point(1000.0, 2000.0, 100.0, origin);
        
        // Basic properties
        CHECK(enu_point.x == 1000.0);
        CHECK(enu_point.y == 2000.0);
        CHECK(enu_point.z == 100.0);
        CHECK(enu_point.datum.lat == origin.lat);
        
        // Distance calculations
        ENU other_point(1100.0, 2000.0, 100.0, origin);
        double distance = enu_point.distance_to(other_point);
        CHECK(std::abs(distance - 100.0) < 0.01);  // Should be 100m apart
    }
    
    SUBCASE("Point arithmetic operations") {
        Point p1(10.0, 20.0, 30.0);
        Point p2(5.0, 10.0, 15.0);
        
        // Addition
        Point sum = p1 + p2;
        CHECK(sum.x == 15.0);
        CHECK(sum.y == 30.0);
        CHECK(sum.z == 45.0);
        
        // Subtraction
        Point diff = p1 - p2;
        CHECK(diff.x == 5.0);
        CHECK(diff.y == 10.0);
        CHECK(diff.z == 15.0);
        
        // Scaling
        Point scaled = p1 * 2.0;
        CHECK(scaled.x == 20.0);
        CHECK(scaled.y == 40.0);
        CHECK(scaled.z == 60.0);
        
        // Distance
        double dist = p1.distance_to(p2);
        CHECK(std::abs(dist - std::sqrt(5*5 + 10*10 + 15*15)) < 0.01);
    }
}

TEST_CASE("Spatial indexing and queries") {
    
    SUBCASE("Spatial hash grid operations") {
        concord::indexing::SpatialHashGrid<int> grid(10.0);  // 10-unit cells
        
        // Insert test data
        Point p1(15.0, 25.0, 0.0);
        Point p2(18.0, 22.0, 0.0);
        Point p3(50.0, 60.0, 0.0);  // Far away
        
        grid.insert(p1, 100);
        grid.insert(p2, 200);
        grid.insert(p3, 300);
        
        // Query nearby points
        auto nearby = grid.query(Point(16.0, 24.0, 0.0), 5.0);
        
        // Should find the two nearby points
        CHECK(nearby.size() >= 2);
        
        // Query with larger radius
        auto all_points = grid.query(Point(30.0, 40.0, 0.0), 50.0);
        CHECK(all_points.size() >= 3);
    }
    
    SUBCASE("Distance-based spatial queries") {
        // Create test points in a known pattern
        std::vector<Point> test_points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(10.0, 10.0, 0.0)  // Far point
        };
        
        Point query_point(0.5, 0.5, 0.0);
        
        // Count points within different radii
        int count_small = 0, count_large = 0;
        for (const auto& pt : test_points) {
            double dist = query_point.distance_to(pt);
            if (dist <= 1.0) count_small++;
            if (dist <= 15.0) count_large++;
        }
        
        CHECK(count_small >= 3);  // Should find 3 nearby points
        CHECK(count_large == 4);  // Should find all points
    }
}

TEST_CASE("Mathematical operations and transformations") {
    
    SUBCASE("Vector and matrix operations") {
        // Test Vec3d operations
        Vec3d v1;
        v1[0] = 1.0; v1[1] = 2.0; v1[2] = 3.0;
        
        Vec3d v2;
        v2[0] = 4.0; v2[1] = 5.0; v2[2] = 6.0;
        
        Vec3d sum = v1 + v2;
        CHECK(sum[0] == 5.0);
        CHECK(sum[1] == 7.0);
        CHECK(sum[2] == 9.0);
        
        // Dot product
        double dot = dot_product(v1, v2);
        CHECK(std::abs(dot - 32.0) < 0.01);  // 1*4 + 2*5 + 3*6 = 32
        
        // Cross product
        Vec3d cross = cross_product(v1, v2);
        CHECK(std::abs(cross[0] - (-3.0)) < 0.01);
        CHECK(std::abs(cross[1] - 6.0) < 0.01);
        CHECK(std::abs(cross[2] - (-3.0)) < 0.01);
    }
    
    SUBCASE("Quaternion operations") {
        // Test quaternion creation and normalization
        Quaternion q(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
        Quaternion normalized_q = q.normalized();
        
        CHECK(std::abs(normalized_q.norm() - 1.0) < 0.01);
        
        // Test quaternion multiplication (identity should preserve)
        Quaternion q2(0.0, 1.0, 0.0, 0.0);
        Quaternion result = normalized_q * q2;
        
        // Result should be q2 since normalized_q is identity
        CHECK(std::abs(result.x - q2.x) < 0.01);
        CHECK(std::abs(result.y - q2.y) < 0.01);
        CHECK(std::abs(result.z - q2.z) < 0.01);
    }
}

TEST_CASE("Geometric shapes and algorithms") {
    
    SUBCASE("Circle geometry") {
        Point center(0.0, 0.0, 0.0);
        double radius = 5.0;
        Circle circle(center, radius);
        
        // Test basic properties
        CHECK(circle.getRadius() == radius);
        CHECK(circle.getCenter().x == center.x);
        
        // Test area calculation
        double expected_area = M_PI * radius * radius;
        CHECK(std::abs(circle.area() - expected_area) < 0.01);
        
        // Test point containment
        Point inside(1.0, 1.0, 0.0);
        Point outside(10.0, 10.0, 0.0);
        CHECK(circle.contains(inside));
        CHECK_FALSE(circle.contains(outside));
    }
    
    SUBCASE("Line geometry") {
        Point start(0.0, 0.0, 0.0);
        Point end(3.0, 4.0, 0.0);
        Line line(start, end);
        
        // Test length calculation
        double expected_length = 5.0;  // 3-4-5 triangle
        CHECK(std::abs(line.length() - expected_length) < 0.01);
        
        // Test start and end points
        CHECK(line.getStart().x == start.x);
        CHECK(line.getStart().y == start.y);
        CHECK(line.getEnd().x == end.x);
        CHECK(line.getEnd().y == end.y);
    }
    
    SUBCASE("Bounding box operations") {
        Point min_pt(1.0, 2.0, 3.0);
        Point max_pt(4.0, 6.0, 9.0);
        AABB bbox(min_pt, max_pt);
        
        // Test containment
        Point inside(2.0, 3.0, 5.0);
        Point outside(0.0, 0.0, 0.0);
        CHECK(bbox.contains(inside));
        CHECK_FALSE(bbox.contains(outside));
        
        // Test volume calculation
        double expected_volume = 3.0 * 4.0 * 6.0;  // (4-1) * (6-2) * (9-3)
        CHECK(std::abs(bbox.volume() - expected_volume) < 0.01);
        
        // Test center calculation
        Point center = bbox.center();
        CHECK(std::abs(center.x - 2.5) < 0.01);  // (1+4)/2
        CHECK(std::abs(center.y - 4.0) < 0.01);  // (2+6)/2
        CHECK(std::abs(center.z - 6.0) < 0.01);  // (3+9)/2
    }
}

TEST_CASE("Builder pattern edge cases and robustness") {
    
    SUBCASE("Invalid datum handling") {
        Point point(100.0, 200.0, 50.0);
        
        // Test with extreme datum values
        Datum extreme_datum(89.0, 179.0, 10000.0);
        
        // Should not throw, but handle gracefully
        auto result = convert(point)
            .withDatum(extreme_datum)
            .as<ENU>()
            .get();
        
        // Verify it produced some result
        CHECK(result.datum.lat == extreme_datum.lat);
    }
    
    SUBCASE("Zero distance edge cases") {
        Point origin(0.0, 0.0, 0.0);
        Point same_point(0.0, 0.0, 0.0);
        
        // Distance should be zero
        double distance = origin.distance_to(same_point);
        CHECK(distance == 0.0);
        
        // Magnitude of zero vector
        CHECK(origin.magnitude() == 0.0);
    }
    
    SUBCASE("Large coordinate values") {
        // Test with very large coordinate values
        Point large_point(1e6, 1e6, 1e6);
        Datum datum(45.0, -120.0, 0.0);
        
        auto result = convert(large_point)
            .withDatum(datum)
            .as<ENU>()
            .to<WGS>()
            .get();
        
        // Should produce valid coordinates
        CHECK(std::abs(result.lat) <= 90.0);
        CHECK(std::abs(result.lon) <= 180.0);
    }
    
    SUBCASE("Coordinate system round-trip precision") {
        // Test precision preservation in round-trip conversions
        Datum datum(37.4220, -122.0841, 0.0);
        WGS original_wgs(37.4225, -122.0845, 100.0);
        
        // WGS -> ENU -> WGS round trip
        auto enu_intermediate = convert(original_wgs)
            .withDatum(datum)
            .as<ENU>()
            .get();
            
        auto final_wgs = convert(enu_intermediate)
            .as<WGS>()
            .get();
        
        // Should preserve coordinates within reasonable precision
        CHECK(std::abs(final_wgs.lat - original_wgs.lat) < 1e-10);
        CHECK(std::abs(final_wgs.lon - original_wgs.lon) < 1e-10);
        CHECK(std::abs(final_wgs.alt - original_wgs.alt) < 1e-6);
    }
}
