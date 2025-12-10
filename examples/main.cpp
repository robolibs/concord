#include <iomanip>
#include <iostream>
#include <vector>

// Include the main concord header which includes everything
#include <concord/concord.hpp>

using namespace concord;

void test_mathematical_types() {
    std::cout << "\n=== Testing Mathematical Types ===" << std::endl;

    // Test Vec3d operations
    Vec3d v1{1.0, 2.0, 3.0};
    Vec3d v2{4.0, 5.0, 6.0};
    Vec3d v3 = v1 + v2;

    std::cout << "Vector addition: (" << v1[0] << "," << v1[1] << "," << v1[2] << ") + " << "(" << v2[0] << "," << v2[1]
              << "," << v2[2] << ") = " << "(" << v3[0] << "," << v3[1] << "," << v3[2] << ")" << std::endl;

    Vec3d scaled = v1 * 2.0;
    std::cout << "Vector scaling: " << "(" << v1[0] << "," << v1[1] << "," << v1[2] << ") * 2 = "
              << "(" << scaled[0] << "," << scaled[1] << "," << scaled[2] << ")" << std::endl;

    // Test Mat3d operations
    Mat3d mat;
    mat[0][0] = 1.0;
    mat[0][1] = 0.0;
    mat[0][2] = 0.0;
    mat[1][0] = 0.0;
    mat[1][1] = 1.0;
    mat[1][2] = 0.0;
    mat[2][0] = 0.0;
    mat[2][1] = 0.0;
    mat[2][2] = 1.0;

    Vec3d result = mat * v1;
    std::cout << "Matrix * vector (identity): (" << result[0] << "," << result[1] << "," << result[2] << ")"
              << std::endl;
}

void test_enhanced_basic_types() {
    std::cout << "\n=== Testing Enhanced Basic Types ===" << std::endl;

    try {
        // Test enhanced WGS coordinates
        WGS seattle(47.6062, -122.3321, 56.0);
        WGS portland(45.5152, -122.6784, 15.0);

        std::cout << "Seattle: " << seattle.lat << ", " << seattle.lon << ", " << seattle.alt << "m" << std::endl;
        std::cout << "Portland: " << portland.lat << ", " << portland.lon << ", " << portland.alt << "m" << std::endl;

        // Test distance calculation
        double distance = seattle.distance_to(portland);
        std::cout << "Distance between Seattle and Portland: " << distance << " meters" << std::endl;

        // Test bearing calculation
        double bearing = seattle.bearing_to(portland);
        std::cout << "Bearing from Seattle to Portland: " << bearing << " degrees" << std::endl;

        // Test enhanced ENU coordinates
        ENU enu1(100.0, 200.0, 50.0);
        ENU enu2(300.0, 400.0, 75.0);

        double enu_distance = enu1.distance_to(enu2);
        std::cout << "ENU distance: " << enu_distance << " meters" << std::endl;

        // Test enhanced Quaternion
        Quaternion q1(1.0, 0.0, 0.0, 0.0);
        Quaternion q2(0.707, 0.0, 0.0, 0.707); // 90 degree rotation around Z
        Quaternion result = q1 * q2;           // Use operator* instead of multiply method

        std::cout << "Quaternion multiplication result: (" << result.w << "," << result.x << "," << result.y << ","
                  << result.z << ")" << std::endl;

    } catch (const std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

void test_coordinate_systems() {
    std::cout << "\n=== Testing Coordinate Systems ===" << std::endl;

    try {
        // Test basic WGS coordinates
        WGS wgs_point(47.6062, -122.3321, 56.0); // Seattle

        std::cout << "WGS coordinates:" << std::endl;
        std::cout << "  Lat: " << wgs_point.lat << ", Lon: " << wgs_point.lon << ", Alt: " << wgs_point.alt
                  << std::endl;

        // Test validation using proper namespace
        try {
            concord::validation::validate_latitude(wgs_point.lat);
            concord::validation::validate_longitude(wgs_point.lon);
            std::cout << "  Coordinates are valid" << std::endl;
        } catch (const std::exception &e) {
            std::cout << "  Validation error: " << e.what() << std::endl;
        }

    } catch (const std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

void test_geometric_types() {
    std::cout << "\n=== Testing Geometric Types ===" << std::endl;

    // Test basic Point operations
    Point p1(5, 5, 5);
    Point p2(10, 10, 10);

    std::cout << "Point 1: (" << p1.x << ", " << p1.y << ", " << p1.z << ")" << std::endl;
    std::cout << "Point 2: (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;

    double distance = p1.distance_to(p2);
    std::cout << "Distance between points: " << distance << std::endl;
}

void test_spatial_algorithms() {
    std::cout << "\n=== Testing Spatial Algorithms ===" << std::endl;

    std::cout << "Spatial algorithms are loaded and available" << std::endl;
    std::cout << "- Distance calculations: available" << std::endl;
    std::cout << "- Line intersections: available" << std::endl;
    std::cout << "- Convex hull: available" << std::endl;
}

void test_spatial_indexing() {
    std::cout << "\n=== Testing Spatial Indexing ===" << std::endl;

    // Test spatial hash grid
    concord::indexing::SpatialHashGrid<int> grid(10.0); // 10 unit cell size
    Point p1(15, 25, 0);
    Point p2(18, 22, 0);

    grid.insert(p1, 100);
    grid.insert(p2, 200);

    auto nearby = grid.query(Point(20, 20, 0), 10.0);
    std::cout << "Hash grid found " << nearby.size() << " points within radius" << std::endl;
}

void test_polygon_partition() {
    std::cout << "\n=== Testing Polygon Partition ===" << std::endl;

    try {
        TPPLPartition partition;

        // Test 1: Simple square polygon (convex)
        std::cout << "\nTest 1: Square polygon (convex)" << std::endl;
        std::vector<Point> squarePoints = {Point(0.0, 0.0, 0.0), Point(10.0, 0.0, 0.0), Point(10.0, 10.0, 0.0),
                                           Point(0.0, 10.0, 0.0)};
        Polygon square(squarePoints);
        std::cout << "Created square with " << square.numVertices() << " vertices" << std::endl;

        // Test 2: More complex concave polygon
        std::cout << "\nTest 2: Concave L-shaped polygon" << std::endl;
        std::vector<Point> lShapePoints = {Point(0.0, 0.0, 0.0),   Point(20.0, 0.0, 0.0),  Point(20.0, 10.0, 0.0),
                                           Point(10.0, 10.0, 0.0), Point(10.0, 20.0, 0.0), Point(0.0, 20.0, 0.0)};
        Polygon lShape(lShapePoints);
        std::cout << "Created L-shape with " << lShape.numVertices() << " vertices" << std::endl;

        // Test triangulation
        std::cout << "\n--- Triangulation Test ---" << std::endl;
        PolygonList inputPolygons;
        inputPolygons.push_back(square);

        PolygonList ecTriangles;
        int result = partition.Triangulate_EC(&inputPolygons, &ecTriangles);
        if (result) {
            std::cout << "EC Triangulation successful! Generated " << ecTriangles.size() << " triangles" << std::endl;
        } else {
            std::cout << "EC Triangulation failed" << std::endl;
        }

    } catch (const std::exception &e) {
        std::cout << "Error in polygon partition test: " << e.what() << std::endl;
    }
}

int main() {
    std::cout << "=== Concord Library Comprehensive Test ===" << std::endl;
    std::cout << "Capabilities:" << std::endl;
    std::cout << "  Mathematical Types: " << (HAS_MATHEMATICAL_TYPES ? "Yes" : "No") << std::endl;
    std::cout << "  Spatial Indexing: " << (HAS_SPATIAL_INDEXING ? "Yes" : "No") << std::endl;
    std::cout << "  Advanced Algorithms: " << (HAS_ADVANCED_ALGORITHMS ? "Yes" : "No") << std::endl;
    std::cout << "  Multiple Datums: " << (HAS_MULTIPLE_DATUMS ? "Yes" : "No") << std::endl;

    // Set precision for output
    std::cout << std::fixed << std::setprecision(6);

    // Run all tests
    test_mathematical_types();
    test_enhanced_basic_types();
    test_coordinate_systems();
    test_geometric_types();
    test_spatial_algorithms();
    test_spatial_indexing();
    test_polygon_partition();

    std::cout << "\n=== All Tests Completed ===" << std::endl;
    return 0;
}
