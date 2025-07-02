#include <concord/algorithms/spatial_algorithms.hpp>
#include <concord/indexing/hash_grid/spatial_hash_grid.hpp>
#include <concord/indexing/rtree/rtree.hpp>
#include <concord/indexing/quadtree/quadtree.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("Spatial algorithms") {
    SUBCASE("Point in polygon test") {
        // Create a simple square polygon
        std::vector<Point> square_points;
        Point p;

        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;
        square_points.push_back(p);
        p.x = 10.0;
        p.y = 0.0;
        p.z = 0.0;
        square_points.push_back(p);
        p.x = 10.0;
        p.y = 10.0;
        p.z = 0.0;
        square_points.push_back(p);
        p.x = 0.0;
        p.y = 10.0;
        p.z = 0.0;
        square_points.push_back(p);

        Polygon square(square_points);

        // Test point inside
        Point inside;
        inside.x = 5.0;
        inside.y = 5.0;
        inside.z = 0.0;
        CHECK(square.contains(inside) == true);

        // Test point outside
        Point outside;
        outside.x = 15.0;
        outside.y = 5.0;
        outside.z = 0.0;
        CHECK(square.contains(outside) == false);

        // Test point on edge
        Point on_edge;
        on_edge.x = 0.0;
        on_edge.y = 5.0;
        on_edge.z = 0.0;
        CHECK(square.contains(on_edge) == true);
    }

    SUBCASE("Line intersection") {
        // Create two intersecting lines
        Point line1_start, line1_end, line2_start, line2_end;

        line1_start.x = 0.0;
        line1_start.y = 0.0;
        line1_start.z = 0.0;
        line1_end.x = 10.0;
        line1_end.y = 10.0;
        line1_end.z = 0.0;

        line2_start.x = 0.0;
        line2_start.y = 10.0;
        line2_start.z = 0.0;
        line2_end.x = 10.0;
        line2_end.y = 0.0;
        line2_end.z = 0.0;

        Line line1(line1_start, line1_end);
        Line line2(line2_start, line2_end);

        Point intersection;
        bool intersects = spatial::lineIntersection(line1, line2, intersection);

        CHECK(intersects == true);
        // Lines should intersect at (5, 5)
        CHECK(intersection.x == doctest::Approx(5.0));
        CHECK(intersection.y == doctest::Approx(5.0));
    }

    SUBCASE("Convex hull") {
        // Create a set of points
        std::vector<Point> points;
        Point p;

        // Add points that form a square with some interior points
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;
        points.push_back(p);
        p.x = 10.0;
        p.y = 0.0;
        p.z = 0.0;
        points.push_back(p);
        p.x = 10.0;
        p.y = 10.0;
        p.z = 0.0;
        points.push_back(p);
        p.x = 0.0;
        p.y = 10.0;
        p.z = 0.0;
        points.push_back(p);
        p.x = 5.0;
        p.y = 5.0;
        p.z = 0.0;
        points.push_back(p); // Interior point

        auto hull = spatial::convexHull(points);

        // Hull should have 4 points (the corners)
        CHECK(hull.numVertices() == 4);
    }
}

TEST_CASE("Spatial indexing") {
    SUBCASE("R-Tree insertion and query") {
        concord::indexing::RTree<Point> rtree;

        // Insert some points with their AABB
        Point p1, p2, p3;
        p1.x = 1.0;
        p1.y = 1.0;
        p1.z = 0.0;
        p2.x = 5.0;
        p2.y = 5.0;
        p2.z = 0.0;
        p3.x = 10.0;
        p3.y = 10.0;
        p3.z = 0.0;

        // Create small bounding boxes around each point
        AABB bbox1(p1, p1);
        AABB bbox2(p2, p2);
        AABB bbox3(p3, p3);

        rtree.insert(bbox1, p1);
        rtree.insert(bbox2, p2);
        rtree.insert(bbox3, p3);

        // Query for points near (2, 2)
        Point query_min, query_max;
        query_min.x = 0.0;
        query_min.y = 0.0;
        query_min.z = 0.0;
        query_max.x = 3.0;
        query_max.y = 3.0;
        query_max.z = 1.0;
        AABB query_box(query_min, query_max);

        auto results = rtree.search(query_box);

        // Should find point p1
        CHECK(results.size() >= 1);
    }

    SUBCASE("QuadTree operations") {
        Point boundary_min, boundary_max;
        boundary_min.x = 0.0;
        boundary_min.y = 0.0;
        boundary_min.z = 0.0;
        boundary_max.x = 100.0;
        boundary_max.y = 100.0;
        boundary_max.z = 1.0;
        AABB boundary(boundary_min, boundary_max);

        concord::indexing::QuadTree<Point> qtree(boundary);

        // Insert points
        Point p1, p2, p3;
        p1.x = 25.0;
        p1.y = 25.0;
        p1.z = 0.0;
        p2.x = 75.0;
        p2.y = 25.0;
        p2.z = 0.0;
        p3.x = 25.0;
        p3.y = 75.0;
        p3.z = 0.0;

        qtree.insert(p1, p1);
        qtree.insert(p2, p2);
        qtree.insert(p3, p3);

        // Query a region
        Point query_min, query_max;
        query_min.x = 0.0;
        query_min.y = 0.0;
        query_min.z = 0.0;
        query_max.x = 50.0;
        query_max.y = 50.0;
        query_max.z = 1.0;
        AABB query_region(query_min, query_max);

        auto results = qtree.query(query_region);

        // Should find p1
        CHECK(results.size() >= 1);
    }
}
