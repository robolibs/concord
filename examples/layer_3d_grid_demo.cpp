#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// Include the main concord header which includes everything
#include <concord/concord.hpp>
#include <concord/geometry/layer/layer.hpp>

using namespace concord;

void demonstrate_basic_layer_usage() {
    std::cout << "\n=== Basic Layer 3D Grid Usage ===" << std::endl;

    // Create a 3D layer: 4x4x3 grid with 1.0m XY resolution and 0.5m Z resolution
    Pose origin_pose;
    origin_pose.point = {0.0, 0.0, 0.0};
    origin_pose.angle = {0.0, 0.0, 0.0}; // No rotation

    Layer<double> layer(4, 4, 3, 1.0, 0.5, true, origin_pose);

    std::cout << "Created 3D layer with dimensions: " << layer.rows() << "x" << layer.cols() << "x" << layer.layers()
              << std::endl;
    std::cout << "XY cell size: " << layer.inradius() << "m, Z cell size: " << layer.layer_height() << "m" << std::endl;

    // Fill the layer with some sample data (distance from center)
    Point center_point = layer.get_point(2, 2, 1); // Center of the grid
    std::cout << "Grid center at: (" << center_point.x << ", " << center_point.y << ", " << center_point.z << ")"
              << std::endl;

    for (size_t r = 0; r < layer.rows(); ++r) {
        for (size_t c = 0; c < layer.cols(); ++c) {
            for (size_t l = 0; l < layer.layers(); ++l) {
                Point cell_point = layer.get_point(r, c, l);
                double distance =
                    std::sqrt(std::pow(cell_point.x - center_point.x, 2) + std::pow(cell_point.y - center_point.y, 2) +
                              std::pow(cell_point.z - center_point.z, 2));
                layer(r, c, l) = distance;
            }
        }
    }

    // Display some sample values
    std::cout << "Sample cell values (distance from center):" << std::endl;
    for (size_t l = 0; l < layer.layers(); ++l) {
        std::cout << "Layer " << l << ":" << std::endl;
        for (size_t r = 0; r < 2; ++r) {     // Just show first 2 rows
            for (size_t c = 0; c < 2; ++c) { // Just show first 2 cols
                std::cout << "  [" << r << "," << c << "," << l << "] = " << std::fixed << std::setprecision(2)
                          << layer(r, c, l) << std::endl;
            }
        }
    }
}

void demonstrate_world_coordinate_conversion() {
    std::cout << "\n=== World Coordinate Conversion ===" << std::endl;

    // Create layer with offset
    Pose offset_pose;
    offset_pose.point = {10.0, 5.0, 2.0}; // Translated
    offset_pose.angle = {0.0, 0.0, 0.0};  // No rotation

    Layer<float> layer(3, 3, 2, 2.0, 1.0, true, offset_pose);

    // Set some values
    layer(1, 1, 0) = 100.0f;
    layer(2, 0, 1) = 200.0f;

    std::cout << "Layer positioned at offset: (" << offset_pose.point.x << ", " << offset_pose.point.y << ", "
              << offset_pose.point.z << ")" << std::endl;

    // Test world coordinate access
    Point world_point = layer.get_point(1, 1, 0);
    std::cout << "Cell [1,1,0] is at world coordinates: (" << world_point.x << ", " << world_point.y << ", "
              << world_point.z << ")" << std::endl;

    float value_at_world = layer.get_value_at_world(world_point);
    std::cout << "Value at world point: " << value_at_world << std::endl;

    // Convert back to grid coordinates
    auto [r, c, l] = layer.world_to_grid(world_point);
    std::cout << "World point converts back to grid indices: [" << r << "," << c << "," << l << "]" << std::endl;

    // Set value using world coordinates
    Point another_world_point{12.0, 7.0, 2.5};
    layer.set_value_at_world(another_world_point, 999.0f);

    auto [r2, c2, l2] = layer.world_to_grid(another_world_point);
    std::cout << "Set value 999 at world point (" << another_world_point.x << ", " << another_world_point.y << ", "
              << another_world_point.z << ") -> grid [" << r2 << "," << c2 << "," << l2 << "] = " << layer(r2, c2, l2)
              << std::endl;
}

void demonstrate_layer_slicing() {
    std::cout << "\n=== Layer Slicing and Grid Extraction ===" << std::endl;

    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};

    Layer<int> layer(4, 3, 3, 1.0, 1.0, true, pose);

    // Fill with pattern data: row*100 + col*10 + layer
    for (size_t r = 0; r < layer.rows(); ++r) {
        for (size_t c = 0; c < layer.cols(); ++c) {
            for (size_t l = 0; l < layer.layers(); ++l) {
                layer(r, c, l) = static_cast<int>(r * 100 + c * 10 + l);
            }
        }
    }

    // Extract a 2D grid from layer 1
    auto extracted_grid = layer.extract_grid(1);
    std::cout << "Extracted 2D grid from layer 1:" << std::endl;
    std::cout << "Grid dimensions: " << extracted_grid.rows() << "x" << extracted_grid.cols() << std::endl;

    // Show the extracted grid data
    for (size_t r = 0; r < extracted_grid.rows(); ++r) {
        for (size_t c = 0; c < extracted_grid.cols(); ++c) {
            std::cout << extracted_grid(r, c) << " ";
        }
        std::cout << std::endl;
    }

    // Access layer slices directly
    std::cout << "\nDirect layer slice access:" << std::endl;
    auto layer_2_data = layer.layer(2);
    std::cout << "Layer 2 has " << layer_2_data.size() << " elements" << std::endl;

    // Access row within a specific layer
    auto row_1_layer_0 = layer.row(1, 0);
    std::cout << "Row 1 in layer 0: ";
    for (size_t i = 0; i < row_1_layer_0.size(); ++i) {
        std::cout << row_1_layer_0[i] << " ";
    }
    std::cout << std::endl;
}

void demonstrate_3d_rotation() {
    std::cout << "\n=== 3D Rotation Example ===" << std::endl;

    // Create layer with rotation
    Pose rotated_pose;
    rotated_pose.point = {0.0, 0.0, 0.0};
    rotated_pose.angle = {M_PI / 4, 0.0, 0.0}; // 45-degree yaw rotation

    Layer<double> layer(3, 3, 2, 1.0, 1.0, true, rotated_pose);

    std::cout << "Created layer with 45-degree yaw rotation" << std::endl;

    // Show how rotation affects point coordinates
    std::cout << "Point coordinates with rotation:" << std::endl;
    for (size_t r = 0; r < 2; ++r) {
        for (size_t c = 0; c < 2; ++c) {
            for (size_t l = 0; l < 2; ++l) {
                Point p = layer.get_point(r, c, l);
                std::cout << "[" << r << "," << c << "," << l << "] -> (" << std::fixed << std::setprecision(2) << p.x
                          << ", " << p.y << ", " << p.z << ")" << std::endl;
            }
        }
    }
}

void demonstrate_bulk_operations() {
    std::cout << "\n=== Bulk Operations and Performance ===" << std::endl;

    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};

    Layer<float> layer(10, 10, 5, 0.5, 0.2, true, pose);

    // Fill with random data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 100.0f);

    for (size_t r = 0; r < layer.rows(); ++r) {
        for (size_t c = 0; c < layer.cols(); ++c) {
            for (size_t l = 0; l < layer.layers(); ++l) {
                layer(r, c, l) = dis(gen);
            }
        }
    }

    std::cout << "Created " << layer.rows() << "x" << layer.cols() << "x" << layer.layers() << " layer with random data"
              << std::endl;

    // Bulk point extraction
    auto points_with_data = layer.flatten_points_with_data();
    std::cout << "Extracted " << points_with_data.size() << " points with data" << std::endl;

    // Show statistics
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();
    float sum = 0.0f;

    for (const auto &point : points_with_data) {
        float data_val = point[3]; // 4th element is the data value
        min_val = std::min(min_val, data_val);
        max_val = std::max(max_val, data_val);
        sum += data_val;
    }

    std::cout << "Data statistics:" << std::endl;
    std::cout << "  Min: " << std::fixed << std::setprecision(2) << min_val << std::endl;
    std::cout << "  Max: " << max_val << std::endl;
    std::cout << "  Mean: " << sum / points_with_data.size() << std::endl;

    // Show corner points
    auto corners = layer.corners();
    std::cout << "Layer corner points:" << std::endl;
    for (size_t i = 0; i < corners.size(); ++i) {
        std::cout << "  Corner " << i << ": (" << std::fixed << std::setprecision(2) << corners[i].x << ", "
                  << corners[i].y << ", " << corners[i].z << ")" << std::endl;
    }
}

void demonstrate_reverse_coordinates() {
    std::cout << "\n=== Reverse Coordinate Modes ===" << std::endl;

    Pose pose;
    pose.point = {0.0, 0.0, 0.0};
    pose.angle = {0.0, 0.0, 0.0};

    // Create layers with different coordinate orientations
    Layer<int> normal_layer(3, 3, 3, 1.0, 1.0, true, pose, false, false);
    Layer<int> reverse_y_layer(3, 3, 3, 1.0, 1.0, true, pose, true, false);
    Layer<int> reverse_z_layer(3, 3, 3, 1.0, 1.0, true, pose, false, true);

    std::cout << "Comparing coordinate systems:" << std::endl;

    // Show how reverse coordinates affect point positions
    Point normal_p00 = normal_layer.get_point(0, 0, 0);
    Point reverse_y_p00 = reverse_y_layer.get_point(0, 0, 0);
    Point reverse_z_p00 = reverse_z_layer.get_point(0, 0, 0);

    std::cout << "Point [0,0,0]:" << std::endl;
    std::cout << "  Normal:    (" << normal_p00.x << ", " << normal_p00.y << ", " << normal_p00.z << ")" << std::endl;
    std::cout << "  Reverse Y: (" << reverse_y_p00.x << ", " << reverse_y_p00.y << ", " << reverse_y_p00.z << ")"
              << std::endl;
    std::cout << "  Reverse Z: (" << reverse_z_p00.x << ", " << reverse_z_p00.y << ", " << reverse_z_p00.z << ")"
              << std::endl;

    Point normal_p22 = normal_layer.get_point(2, 2, 2);
    Point reverse_y_p22 = reverse_y_layer.get_point(2, 2, 2);
    Point reverse_z_p22 = reverse_z_layer.get_point(2, 2, 2);

    std::cout << "Point [2,2,2]:" << std::endl;
    std::cout << "  Normal:    (" << normal_p22.x << ", " << normal_p22.y << ", " << normal_p22.z << ")" << std::endl;
    std::cout << "  Reverse Y: (" << reverse_y_p22.x << ", " << reverse_y_p22.y << ", " << reverse_y_p22.z << ")"
              << std::endl;
    std::cout << "  Reverse Z: (" << reverse_z_p22.x << ", " << reverse_z_p22.y << ", " << reverse_z_p22.z << ")"
              << std::endl;
}

int main() {
    std::cout << "Concord Layer 3D Grid Demonstration" << std::endl;
    std::cout << "====================================" << std::endl;

    try {
        demonstrate_basic_layer_usage();
        demonstrate_world_coordinate_conversion();
        demonstrate_layer_slicing();
        demonstrate_3d_rotation();
        demonstrate_bulk_operations();
        demonstrate_reverse_coordinates();

        std::cout << "\n=== Layer Demo Complete ===" << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
