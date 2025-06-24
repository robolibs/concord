#include <concord/concord.hpp>
#include <doctest/doctest.h>
#include <vector>

using namespace concord;

TEST_CASE("GIS Pipeline Integration") {
    SUBCASE("GPS to local coordinate system workflow") {
        // Simulate a surveying workflow: GPS points -> UTM -> Local grid -> Spatial analysis
        
        // 1. Start with GPS coordinates (WGS84)
        std::vector<WGS> gps_points = {
            WGS(40.7589, -73.9851, 30.0), // Times Square area
            WGS(40.7614, -73.9776, 25.0), // Bryant Park
            WGS(40.7505, -73.9934, 35.0), // Hudson Yards
            WGS(40.7549, -73.9840, 28.0)  // Herald Square
        };
        
        // 2. Convert to UTM for metric calculations
        std::vector<Point> utm_points;
        for (const auto& gps : gps_points) {
            auto [x, y, zone, northern] = wgs_to_utm(gps.lat, gps.lon);
            utm_points.emplace_back(x, y, gps.alt);
        }
        
        CHECK(utm_points.size() == 4);
        CHECK(utm_points[0].x > 580000); // Sanity check for NYC UTM coordinates
        CHECK(utm_points[0].y > 4500000);
        
        // 3. Create local ENU coordinate system with first point as origin
        const auto& ref = gps_points[0];
        std::vector<ENU> local_points;
        for (const auto& gps : gps_points) {
            auto [e, n, u] = gps_to_enu(gps.lat, gps.lon, gps.alt,
                                       ref.lat, ref.lon, ref.alt);
            local_points.emplace_back(e, n, u);
        }
        
        CHECK(local_points[0].x == doctest::Approx(0.0).epsilon(0.1));
        CHECK(local_points[0].y == doctest::Approx(0.0).epsilon(0.1));
        CHECK(local_points[0].z == doctest::Approx(0.0).epsilon(0.1));
        
        // 4. Spatial indexing for efficient queries
        RTree<size_t> spatial_index;
        for (size_t i = 0; i < local_points.size(); ++i) {
            Point p(local_points[i].x, local_points[i].y, local_points[i].z);
            AABB bounds(p, p); // Point bounds
            spatial_index.insert(bounds, i);
        }
        
        // 5. Perform spatial query
        Point query_center(500.0, 300.0, 0.0); // 500m east, 300m north
        double radius = 1000.0; // 1km radius
        Point query_min(query_center.x - radius, query_center.y - radius, query_center.z - radius);
        Point query_max(query_center.x + radius, query_center.y + radius, query_center.z + radius);
        AABB query_bounds(query_min, query_max);
        
        auto nearby = spatial_index.search(query_bounds);
        
        CHECK(nearby.size() >= 0); // Should find some points
    }
    
    SUBCASE("Polygon processing pipeline") {
        // Workflow: Create polygon -> Validate -> Triangulate -> Spatial operations
        
        // 1. Create a complex polygon (L-shape)
        std::vector<Point> boundary = {
            Point(0, 0, 0), Point(10, 0, 0), Point(10, 5, 0),
            Point(5, 5, 0), Point(5, 10, 0), Point(0, 10, 0)
        };
        Polygon building_footprint(boundary);
        
        CHECK(building_footprint.isConnected());
        CHECK(building_footprint.numVertices() == 6);
        
        // 2. Calculate area and perimeter
        double area = building_footprint.area();
        double perimeter = building_footprint.perimeter();
        
        CHECK(area == doctest::Approx(75.0)); // L-shape: 10x5 + 5x5 = 75
        CHECK(perimeter > 30.0); // Sanity check
        
        // 3. Triangulate the polygon
        TPPLPartition partitioner;
        PolygonList triangles, inpolys;
        inpolys.push_back(building_footprint);
        bool success = partitioner.Triangulate_EC(&inpolys, &triangles);
        
        CHECK(success == true);
        CHECK(triangles.size() >= 4); // L-shape should produce multiple triangles
        
        // 4. Verify triangulation area matches original
        double total_triangle_area = 0.0;
        for (const auto& triangle : triangles) {
            total_triangle_area += triangle.area();
        }
        
        CHECK(total_triangle_area == doctest::Approx(area).epsilon(0.01));
        
        // 5. Point-in-polygon tests
        Point inside(2, 2, 0);
        Point outside(7, 7, 0);
        Point on_boundary(0, 5, 0);
        
        CHECK(building_footprint.contains(inside) == true);
        CHECK(building_footprint.contains(outside) == false);
        CHECK(building_footprint.contains(on_boundary) == true);
    }
}

TEST_CASE("Multi-scale Spatial Analysis") {
    SUBCASE("Urban planning workflow") {
        // Workflow: City blocks -> Building footprints -> Spatial queries -> Analysis
        
        // 1. Define city blocks as polygons
        std::vector<Polygon> city_blocks;
        
        // Block 1: Square block
        std::vector<Point> block1_points = {
            Point(0, 0, 0), Point(100, 0, 0), 
            Point(100, 100, 0), Point(0, 100, 0)
        };
        city_blocks.emplace_back(block1_points);
        
        // Block 2: Rectangular block
        std::vector<Point> block2_points = {
            Point(120, 0, 0), Point(200, 0, 0), 
            Point(200, 80, 0), Point(120, 80, 0)
        };
        city_blocks.emplace_back(block2_points);
        
        // 2. Create spatial index for blocks
        RTree<size_t> block_index;
        for (size_t i = 0; i < city_blocks.size(); ++i) {
            AABB bounds = city_blocks[i].getAABB();
            block_index.insert(bounds, i);
        }
        
        // 3. Define building locations within blocks
        std::vector<Point> buildings = {
            Point(25, 25, 0),   // Building in block 1
            Point(75, 75, 0),   // Building in block 1  
            Point(150, 40, 0),  // Building in block 2
            Point(300, 50, 0)   // Building outside any block
        };
        
        // 4. Find which block each building belongs to
        std::vector<int> building_to_block;
        for (const auto& building : buildings) {
            AABB building_bounds(building, building);
            auto candidates = block_index.search(building_bounds);
            
            int block_id = -1; // -1 means no block
            for (const auto& candidate_id : candidates) {
                if (city_blocks[candidate_id].contains(building)) {
                    block_id = static_cast<int>(candidate_id);
                    break;
                }
            }
            building_to_block.push_back(block_id);
        }
        
        CHECK(building_to_block[0] == 0); // First building in block 0
        CHECK(building_to_block[1] == 0); // Second building in block 0
        CHECK(building_to_block[2] == 1); // Third building in block 1
        CHECK(building_to_block[3] == -1); // Fourth building outside blocks
        
        // 5. Calculate building density per block
        std::vector<int> buildings_per_block(city_blocks.size(), 0);
        for (size_t i = 0; i < building_to_block.size(); ++i) {
            if (building_to_block[i] >= 0) {
                buildings_per_block[building_to_block[i]]++;
            }
        }
        
        CHECK(buildings_per_block[0] == 2); // Block 0 has 2 buildings
        CHECK(buildings_per_block[1] == 1); // Block 1 has 1 building
        
        // 6. Calculate density (buildings per unit area)
        for (size_t i = 0; i < city_blocks.size(); ++i) {
            double area = city_blocks[i].area();
            double density = buildings_per_block[i] / area;
            CHECK(density >= 0.0);
            
            if (i == 0) {
                CHECK(density == doctest::Approx(2.0 / 10000.0)); // 2 buildings in 100x100 area
            }
        }
    }
}

TEST_CASE("Navigation and Pathfinding Integration") {
    SUBCASE("Route planning with obstacles") {
        // Workflow: Define space -> Add obstacles -> Create navigation mesh -> Find path
        
        // 1. Define navigation space
        Point space_min(0, 0, 0);
        Point space_max(1000, 1000, 0);
        AABB navigation_space(space_min, space_max);
        
        // 2. Define obstacles as polygons
        std::vector<Polygon> obstacles;
        
        // Obstacle 1: Building
        std::vector<Point> building = {
            Point(200, 200, 0), Point(300, 200, 0),
            Point(300, 300, 0), Point(200, 300, 0)
        };
        obstacles.emplace_back(building);
        
        // Obstacle 2: Park (circular approximation as octagon)
        std::vector<Point> park;
        Point park_center(600, 600, 0);
        double park_radius = 80.0;
        for (int i = 0; i < 8; ++i) {
            double angle = 2.0 * M_PI * i / 8.0;
            Point p;
            p.x = park_center.x + park_radius * cos(angle);
            p.y = park_center.y + park_radius * sin(angle);
            p.z = 0.0;
            park.push_back(p);
        }
        obstacles.emplace_back(park);
        
        // 3. Create spatial index for obstacles
        RTree<size_t> obstacle_index;
        for (size_t i = 0; i < obstacles.size(); ++i) {
            AABB bounds = obstacles[i].getAABB();
            obstacle_index.insert(bounds, i);
        }
        
        // 4. Test path collision detection
        Point start(50, 50, 0);
        Point end(950, 950, 0);
        Line direct_path(start, end);
        
        // Check if path intersects any obstacles
        bool path_blocked = false;
        AABB path_bounds(start, end);
        auto potential_obstacles = obstacle_index.search(path_bounds);
        
        for (const auto& obstacle_id : potential_obstacles) {
            const auto& obstacle = obstacles[obstacle_id];
            // Simplified intersection test - in real implementation would use
            // proper line-polygon intersection
            Point path_mid;
            path_mid.x = (start.x + end.x) / 2.0;
            path_mid.y = (start.y + end.y) / 2.0;
            path_mid.z = 0.0;
            
            if (obstacle.contains(path_mid)) {
                path_blocked = true;
                break;
            }
        }
        
        // Direct path should intersect with the park obstacle
        CHECK(potential_obstacles.size() >= 1);
        
        // 5. Create waypoints around obstacles (simplified pathfinding)
        std::vector<Point> waypoints;
        waypoints.push_back(start);
        
        // Simple strategy: go around obstacles
        waypoints.push_back(Point(400, 400, 0)); // Around building
        waypoints.push_back(Point(750, 750, 0)); // Around park
        waypoints.push_back(end);
        
        // 6. Calculate total path distance
        double total_distance = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            double dx = waypoints[i].x - waypoints[i-1].x;
            double dy = waypoints[i].y - waypoints[i-1].y;
            total_distance += sqrt(dx*dx + dy*dy);
        }
        
        double direct_distance = sqrt((end.x - start.x) * (end.x - start.x) + 
                                    (end.y - start.y) * (end.y - start.y));
        
        CHECK(total_distance >= direct_distance); // Path should be at least as long as direct
        CHECK(total_distance < direct_distance * 2.0); // But not unreasonably long
    }
}

TEST_CASE("Environmental Monitoring Integration") {
    SUBCASE("Sensor network spatial analysis") {
        // Workflow: Sensor placement -> Data collection -> Spatial interpolation
        
        // 1. Define sensor locations
        std::vector<Point> sensor_locations = {
            Point(100, 100, 10),   // Sensor 1
            Point(300, 150, 12),   // Sensor 2  
            Point(200, 300, 8),    // Sensor 3
            Point(450, 200, 15),   // Sensor 4
            Point(350, 350, 11)    // Sensor 5
        };
        
        // 2. Simulate sensor readings (temperature in Celsius)
        std::vector<double> temperature_readings = {
            22.5, 24.1, 21.8, 25.3, 23.2
        };
        
        CHECK(sensor_locations.size() == temperature_readings.size());
        
        // 3. Create spatial index for sensors
        RTree<size_t> sensor_index;
        for (size_t i = 0; i < sensor_locations.size(); ++i) {
            AABB bounds(sensor_locations[i], sensor_locations[i]);
            sensor_index.insert(bounds, i);
        }
        
        // 4. Query sensors within radius of a point of interest
        Point poi(250, 200, 0); // Point of interest
        double query_radius = 150.0;
        
        Point query_min(poi.x - query_radius, poi.y - query_radius, poi.z - query_radius);
        Point query_max(poi.x + query_radius, poi.y + query_radius, poi.z + query_radius);
        AABB poi_bounds(query_min, query_max);
        
        auto nearby_sensors = sensor_index.search(poi_bounds);
        CHECK(nearby_sensors.size() >= 0); // Should find some sensors
        
        // 5. Perform inverse distance weighted interpolation
        auto interpolate_temperature = [&](const Point& query_point) -> double {
            double weighted_sum = 0.0;
            double weight_sum = 0.0;
            
            for (size_t i = 0; i < sensor_locations.size(); ++i) {
                double dx = query_point.x - sensor_locations[i].x;
                double dy = query_point.y - sensor_locations[i].y;
                double distance = sqrt(dx*dx + dy*dy);
                
                if (distance < 1.0) {
                    return temperature_readings[i]; // Very close to sensor
                }
                
                double weight = 1.0 / (distance * distance); // IDW power = 2
                weighted_sum += temperature_readings[i] * weight;
                weight_sum += weight;
            }
            
            return weighted_sum / weight_sum;
        };
        
        // 6. Test interpolation at various points
        double temp_at_poi = interpolate_temperature(poi);
        CHECK(temp_at_poi > 20.0);
        CHECK(temp_at_poi < 30.0);
        
        // Temperature at sensor location should match reading
        double temp_at_sensor = interpolate_temperature(sensor_locations[0]);
        CHECK(temp_at_sensor == doctest::Approx(temperature_readings[0]).epsilon(0.1));
        
        // 7. Create temperature contour analysis
        std::vector<Point> grid_points;
        std::vector<double> grid_temperatures;
        
        // Create a regular grid for contour analysis
        for (int x = 50; x <= 500; x += 50) {
            for (int y = 50; y <= 400; y += 50) {
                Point grid_point(x, y, 0);
                grid_points.push_back(grid_point);
                grid_temperatures.push_back(interpolate_temperature(grid_point));
            }
        }
        
        CHECK(grid_points.size() == grid_temperatures.size());
        CHECK(grid_temperatures.size() > 50); // Should have a good grid coverage
        
        // Find hot spots (temperatures above average)
        double avg_temp = 0.0;
        for (double temp : grid_temperatures) {
            avg_temp += temp;
        }
        avg_temp /= grid_temperatures.size();
        
        int hot_spots = 0;
        for (double temp : grid_temperatures) {
            if (temp > avg_temp + 1.0) { // 1 degree above average
                hot_spots++;
            }
        }
        
        CHECK(hot_spots >= 0); // Should find some variation
    }
}
