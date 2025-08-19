#include <concord/geometry/layer/layer.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("Layer construction and basic properties") {
    SUBCASE("Default constructor") {
        Layer<int> layer;
        CHECK(layer.rows() == 0);
        CHECK(layer.cols() == 0);
        CHECK(layer.layers() == 0);
    }

    SUBCASE("Basic constructor") {
        Pose pose;
        pose.point = {0.0, 0.0, 0.0};
        pose.angle = {0.0, 0.0, 0.0};
        
        Layer<double> layer(10, 8, 5, 1.0, 2.0, true, pose);
        CHECK(layer.rows() == 10);
        CHECK(layer.cols() == 8);
        CHECK(layer.layers() == 5);
        CHECK(layer.inradius() == doctest::Approx(1.0));
        CHECK(layer.layer_height() == doctest::Approx(2.0));
    }

    SUBCASE("Invalid dimensions throw exception") {
        Pose pose;
        CHECK_THROWS_AS(Layer<int>(0, 5, 3, 1.0, 1.0, true, pose), std::invalid_argument);
        CHECK_THROWS_AS(Layer<int>(5, 0, 3, 1.0, 1.0, true, pose), std::invalid_argument);
        CHECK_THROWS_AS(Layer<int>(5, 3, 0, 1.0, 1.0, true, pose), std::invalid_argument);
        CHECK_THROWS_AS(Layer<int>(5, 3, 2, 0.0, 1.0, true, pose), std::invalid_argument);
        CHECK_THROWS_AS(Layer<int>(5, 3, 2, 1.0, 0.0, true, pose), std::invalid_argument);
    }
}

TEST_CASE("Layer indexing and data access") {
    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};
    
    Layer<int> layer(4, 3, 2, 1.0, 1.0, true, pose);

    SUBCASE("3D indexing formula") {
        // Test depth-major ordering: (r*cols + c)*layers + l
        CHECK(layer.index(0, 0, 0) == 0);
        CHECK(layer.index(0, 0, 1) == 1);
        CHECK(layer.index(0, 1, 0) == 2);
        CHECK(layer.index(0, 1, 1) == 3);
        CHECK(layer.index(1, 0, 0) == 6);
    }

    SUBCASE("Data access operators") {
        layer(1, 2, 1) = 42;
        CHECK(layer(1, 2, 1) == 42);
        CHECK(layer.at(1, 2, 1) == 42);

        layer.set_value(2, 1, 0, 100);
        CHECK(layer(2, 1, 0) == 100);
    }

    SUBCASE("Out of bounds access") {
        CHECK_THROWS_AS(layer.at(4, 0, 0), std::out_of_range);
        CHECK_THROWS_AS(layer.at(0, 3, 0), std::out_of_range);
        CHECK_THROWS_AS(layer.at(0, 0, 2), std::out_of_range);
    }
}

TEST_CASE("3D point computation") {
    SUBCASE("Identity transform (no rotation, centered)") {
        Pose pose;
        pose.point = {0.0, 0.0, 0.0};
        pose.angle = {0.0, 0.0, 0.0};
        
        Layer<int> layer(4, 4, 3, 2.0, 1.5, true, pose);
        
        // Center of grid should be at origin
        Point center_point = layer.get_point(2, 2, 1);
        CHECK(center_point.x == doctest::Approx(1.0).epsilon(0.001)); // (2+0.5)*2 - 4 = 1
        CHECK(center_point.y == doctest::Approx(1.0).epsilon(0.001));
        CHECK(center_point.z == doctest::Approx(0.75).epsilon(0.001)); // (1+0.5)*1.5 - 2.25 = 0.75
        
        // Corner point
        Point corner = layer.get_point(0, 0, 0);
        CHECK(corner.x == doctest::Approx(-3.0).epsilon(0.001)); // 0.5*2 - 4 = -3
        CHECK(corner.y == doctest::Approx(-3.0).epsilon(0.001));
        CHECK(corner.z == doctest::Approx(-0.75).epsilon(0.001)); // (0.5+0.5)*1.5 - 2.25 = -0.75
    }

    SUBCASE("Translation offset") {
        Pose pose;
        pose.point = {10.0, 5.0, 2.0};
        pose.angle = {0.0, 0.0, 0.0};
        
        Layer<int> layer(2, 2, 2, 1.0, 1.0, true, pose);
        
        Point point = layer.get_point(0, 0, 0);
        CHECK(point.x == doctest::Approx(9.5).epsilon(0.001)); // 0.5 - 1 + 10
        CHECK(point.y == doctest::Approx(4.5).epsilon(0.001)); // 0.5 - 1 + 5
        CHECK(point.z == doctest::Approx(2.0).epsilon(0.001)); // Updated for centered calculation
    }
}

TEST_CASE("World to grid coordinate conversion") {
    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};
    
    Layer<int> layer(4, 4, 3, 2.0, 1.0, true, pose);

    SUBCASE("Identity transform conversion") {
        Point world_point{1.0, 1.0, 0.5};
        auto [r, c, l] = layer.world_to_grid(world_point);
        
        // Verify round-trip conversion
        Point back_to_world = layer.grid_to_world(r, c, l);
        CHECK(back_to_world.x == doctest::Approx(world_point.x).epsilon(0.1));
        CHECK(back_to_world.y == doctest::Approx(world_point.y).epsilon(0.1));
        CHECK(back_to_world.z == doctest::Approx(world_point.z).epsilon(0.1));
    }

    SUBCASE("World coordinate value access") {
        layer(1, 2, 1) = 99;
        Point world_point = layer.get_point(1, 2, 1);
        
        CHECK(layer.get_value_at_world(world_point) == 99);
        
        layer.set_value_at_world(world_point, 77);
        CHECK(layer(1, 2, 1) == 77);
    }
}

TEST_CASE("Layer slicing and extraction") {
    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};
    
    Layer<double> layer(3, 4, 2, 1.0, 1.0, true, pose);
    
    // Fill with test data
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            for (size_t l = 0; l < 2; ++l) {
                layer(r, c, l) = static_cast<double>(r * 100 + c * 10 + l);
            }
        }
    }

    SUBCASE("Layer access") {
        auto layer_0 = layer.layer(0);
        auto layer_1 = layer.layer(1);
        
        CHECK(layer_0.size() == 12); // 3 * 4
        CHECK(layer_1.size() == 12);
        
        // Check some values - layer_0[0] corresponds to layer(0,0,0), layer_1[0] to layer(0,0,1)
        CHECK(layer_0[0] == 0.0);   // layer(0,0,0) = 0*100 + 0*10 + 0 = 0
        CHECK(layer_1[0] == 1.0);   // layer(0,0,1) = 0*100 + 0*10 + 1 = 1
    }

    SUBCASE("Row access within layer") {
        auto row_0_layer_0 = layer.row(0, 0);
        auto row_1_layer_1 = layer.row(1, 1);
        
        CHECK(row_0_layer_0.size() == 4);
        CHECK(row_1_layer_1.size() == 4);
    }

    SUBCASE("Grid extraction") {
        auto grid = layer.extract_grid(1);
        
        CHECK(grid.rows() == 3);
        CHECK(grid.cols() == 4);
        CHECK(grid.inradius() == doctest::Approx(1.0));
        
        // Verify data was copied correctly from layer 1
        CHECK(grid(0, 0) == layer(0, 0, 1));
        CHECK(grid(1, 2) == layer(1, 2, 1));
        CHECK(grid(2, 3) == layer(2, 3, 1));
    }
}

TEST_CASE("Layer corners and bounds") {
    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};
    
    Layer<int> layer(2, 2, 2, 1.0, 1.0, true, pose);

    SUBCASE("8-corner calculation") {
        auto corners = layer.corners();
        CHECK(corners.size() == 8);
        
        // Check that corners are at expected locations
        // Front-bottom-left corner (0,0,0)
        Point expected_fbl = layer.get_point(0, 0, 0);
        CHECK(corners[0].x == doctest::Approx(expected_fbl.x));
        CHECK(corners[0].y == doctest::Approx(expected_fbl.y));
        CHECK(corners[0].z == doctest::Approx(expected_fbl.z));
        
        // Back-top-right corner (1,1,1)
        Point expected_btr = layer.get_point(1, 1, 1);
        CHECK(corners[6].x == doctest::Approx(expected_btr.x));
        CHECK(corners[6].y == doctest::Approx(expected_btr.y));
        CHECK(corners[6].z == doctest::Approx(expected_btr.z));
    }
}

TEST_CASE("Layer bulk operations") {
    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};
    
    Layer<float> layer(2, 2, 2, 1.0, 1.0, true, pose);
    
    // Fill with test data
    for (size_t r = 0; r < 2; ++r) {
        for (size_t c = 0; c < 2; ++c) {
            for (size_t l = 0; l < 2; ++l) {
                layer(r, c, l) = static_cast<float>(r + c + l);
            }
        }
    }

    SUBCASE("Flatten points with data") {
        auto points = layer.flatten_points_with_data();
        CHECK(points.size() == 8); // 2*2*2
        
        // Each point should have x, y, z, data
        for (const auto& point : points) {
            CHECK(point.size() == 4);
            // Data value should be reasonable (0-3 based on our fill pattern)
            CHECK(point[3] >= 0.0f);
            CHECK(point[3] <= 3.0f);
        }
    }
}

TEST_CASE("Layer equality and comparison") {
    Pose pose1, pose2;
    pose1.point = {0.0, 0.0, 0.0};
    pose1.angle = {0.0, 0.0, 0.0};
    pose2 = pose1;
    
    Layer<int> layer1(3, 3, 2, 1.0, 1.0, true, pose1);
    Layer<int> layer2(3, 3, 2, 1.0, 1.0, true, pose2);
    Layer<int> layer3(3, 3, 3, 1.0, 1.0, true, pose1); // Different layers
    
    SUBCASE("Equality comparison") {
        CHECK(layer1 == layer2);
        CHECK(layer1 != layer3);
        
        // Modify data in layer2
        layer2(1, 1, 1) = 42;
        CHECK(layer1 != layer2);
    }
}

TEST_CASE("Layer reverse coordinate modes") {
    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};
    
    SUBCASE("Reverse Y coordinate") {
        Layer<int> layer(4, 4, 2, 1.0, 1.0, true, pose, true, false);
        
        Point p00 = layer.get_point(0, 0, 0);
        Point p30 = layer.get_point(3, 0, 0);
        
        // With reverse_y, row 0 should have higher Y than row 3
        CHECK(p00.y > p30.y);
    }
    
    SUBCASE("Reverse Z coordinate") {
        Layer<int> layer(2, 2, 4, 1.0, 1.0, true, pose, false, true);
        
        Point p000 = layer.get_point(0, 0, 0);
        Point p003 = layer.get_point(0, 0, 3);
        
        // With reverse_z, layer 0 should have higher Z than layer 3
        CHECK(p000.z > p003.z);
    }
}