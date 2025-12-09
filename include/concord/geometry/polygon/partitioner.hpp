#pragma once

#include "../../algorithms/spatial_algorithms.hpp"
#include "../../types/point.hpp"
#include "partition.hpp"
#include "polygon.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace concord {

    class Partitioner {
      public:
        struct PartitionCriteria {
            double max_area;              // Maximum area in sq meters (5 hectares)
            double min_convexity;         // Minimum convexity ratio (60%)
            double max_aspect_ratio;      // Maximum length/width ratio
            double min_bridge_width;      // Minimum bridge width in meters
            double tooth_threshold;       // Tooth size relative to main body
            double simplify_tolerance;    // Polygon simplification tolerance
            int max_recursion_depth;      // Maximum recursion levels
            bool enable_bridge_detection; // Enable narrow bridge splitting
            bool enable_tooth_detection;  // Enable tooth/extension splitting
            bool enable_aspect_splitting; // Enable elongated field splitting

            // Default constructor
            PartitionCriteria()
                : max_area(50000.0), min_convexity(0.6), max_aspect_ratio(4.0), min_bridge_width(20.0),
                  tooth_threshold(0.3), simplify_tolerance(2.0), max_recursion_depth(5), enable_bridge_detection(true),
                  enable_tooth_detection(true), enable_aspect_splitting(true) {}
        };

      private:
        Polygon border_;
        PartitionCriteria criteria_;

      public:
        std::vector<Polygon> polygons_;

        Partitioner() = default;

        Partitioner(const Polygon &poly) : border_(poly) {}

        // Main partitioning function with intelligent multi-criteria splitting
        std::vector<Polygon> partition(double area_threshold, const PartitionCriteria &criteria = PartitionCriteria{}) {
            criteria_ = criteria;
            criteria_.max_area = area_threshold; // Override with provided threshold

            polygons_.clear();
            std::vector<Polygon> result;

            // Start recursive partitioning
            partition_recursive(border_, result, 0);

            polygons_ = result;
            return result;
        }

        // Partition into approximately equal areas
        std::vector<Polygon> partition_equal_areas(double target_area, double tolerance = 0.2) {
            polygons_.clear();
            std::vector<Polygon> result;

            double total_area = border_.area();
            std::cout << "Total area to partition: " << total_area << " sq.m" << std::endl;

            // Calculate optimal number of parts
            int num_parts = std::max(1, static_cast<int>(std::round(total_area / target_area)));
            std::cout << "Target area per part: " << target_area << " sq.m, need " << num_parts << " parts"
                      << std::endl;

            // For a single part, just return the original
            if (num_parts == 1) {
                result.push_back(border_);
                polygons_ = result;
                return result;
            }

            // Calculate grid dimensions for roughly square parts
            int grid_cols = static_cast<int>(std::ceil(std::sqrt(num_parts)));
            int grid_rows = static_cast<int>(std::ceil(static_cast<double>(num_parts) / grid_cols));

            std::cout << "Using grid: " << grid_rows << " rows x " << grid_cols << " cols" << std::endl;

            // For small number of parts, use simpler splitting
            std::vector<Polygon> grid_parts;
            std::cout << "DEBUG: num_parts = " << num_parts << ", condition = " << (num_parts <= 4) << std::endl;
            if (num_parts <= 4) {
                std::cout << "Using area-based equal splitting for " << num_parts << " parts" << std::endl;
                // Use area-based splitting which is more predictable
                grid_parts = split_by_area_equal(border_, num_parts);
                std::cout << "Area-based splitting returned " << grid_parts.size() << " parts" << std::endl;
            } else {
                std::cout << "Using grid splitting for " << num_parts << " parts" << std::endl;
                // Use split_by_grid with calculated dimensions
                grid_parts = split_by_grid(border_, std::max(grid_rows, grid_cols));
            }

            // Post-process to merge small parts and balance areas
            result = balance_part_areas(grid_parts, target_area, tolerance);

            polygons_ = result;
            return result;
        }

        // Get current partitioning criteria
        const PartitionCriteria &getCriteria() const { return criteria_; }

        // Set partitioning criteria
        void setCriteria(const PartitionCriteria &criteria) { criteria_ = criteria; }

      private:
        // Recursive partitioning with multiple criteria
        void partition_recursive(const Polygon &poly, std::vector<Polygon> &result, int depth) {

            double area = poly.area();

            // Always ensure we handle large polygons, even at max recursion
            if (depth >= criteria_.max_recursion_depth) {
                // When we've reached max recursion depth, force direct grid partitioning
                // to handle remaining large polygons
                if (area > criteria_.max_area) {
                    // Calculate how many rows/columns needed
                    int divisions = std::ceil(std::sqrt(area / criteria_.max_area)) + 1;

                    // Use aggressive grid partitioning (more divisions)
                    std::vector<Polygon> final_splits = split_by_grid(poly, divisions);

                    // Direct addition without further recursion
                    for (const auto &part : final_splits) {
                        // Safety check - only add parts that are valid
                        if (part.getPoints().size() >= 3) {
                            result.push_back(part);
                        }
                    }
                    return;
                }

                // If we're at max depth but polygon is small enough, just add it
                result.push_back(poly);
                return;
            }

            // Check if polygon meets all criteria
            if (meets_all_criteria(poly, area)) {
                result.push_back(poly);
                return;
            }

            // Try different splitting strategies in order of priority
            std::vector<Polygon> split_result;

            // 1. Area-based splitting (highest priority)
            if (area > criteria_.max_area) {
                // For very large polygons, use grid partitioning with enough divisions
                // to ensure all parts are under the area threshold
                if (area > 2 * criteria_.max_area) {
                    int divisions = std::ceil(std::sqrt(area / criteria_.max_area)) + 1;
                    split_result = split_by_grid(poly, divisions);
                } else {
                    split_result = split_by_area(poly);

                    // Check if the basic area-based split worked effectively
                    bool all_under_threshold = true;
                    for (const auto &part : split_result) {
                        if (part.area() > criteria_.max_area) {
                            all_under_threshold = false;
                            break;
                        }
                    }

                    // If not all parts are under threshold, try grid partitioning
                    if (!all_under_threshold && split_result.size() > 0) {
                        int divisions = std::ceil(std::sqrt(area / criteria_.max_area)) + 1;
                        split_result = split_by_grid(poly, divisions);
                    }
                }

                // Process the resulting parts
                if (split_result.size() > 1) {
                    for (const auto &part : split_result) {
                        partition_recursive(part, result, depth + 1);
                    }
                    return;
                }
            }

            // 2. Bridge detection and splitting
            if (criteria_.enable_bridge_detection) {
                split_result = split_by_narrow_bridges(poly);
                if (split_result.size() > 1) {
                    for (const auto &part : split_result) {
                        partition_recursive(part, result, depth + 1);
                    }
                    return;
                }
            }

            // 3. Tooth/extension detection and splitting
            if (criteria_.enable_tooth_detection) {
                split_result = split_by_teeth_and_extensions(poly);
                if (split_result.size() > 1) {
                    for (const auto &part : split_result) {
                        partition_recursive(part, result, depth + 1);
                    }
                    return;
                }
            }

            // 4. Aspect ratio-based splitting
            if (criteria_.enable_aspect_splitting) {
                split_result = split_by_aspect_ratio(poly);
                if (split_result.size() > 1) {
                    for (const auto &part : split_result) {
                        partition_recursive(part, result, depth + 1);
                    }
                    return;
                }
            }

            // 5. Convexity-based splitting (last resort)
            split_result = split_by_convexity(poly);
            if (split_result.size() > 1) {
                for (const auto &part : split_result) {
                    partition_recursive(part, result, depth + 1);
                }
                return;
            }

            // If no splitting worked, accept the polygon as-is
            result.push_back(poly);
        }

        // Check if polygon meets all partitioning criteria
        bool meets_all_criteria(const Polygon &poly, double area) const {
            // Area check
            if (area > criteria_.max_area) {
                return false;
            }

            // Convexity check
            if (calculate_convexity_ratio(poly) < criteria_.min_convexity) {
                return false;
            }

            // Aspect ratio check
            if (calculate_aspect_ratio(poly) > criteria_.max_aspect_ratio) {
                return false;
            }

            return true;
        }

        // Calculate convexity ratio (area / convex_hull_area)
        double calculate_convexity_ratio(const Polygon &poly) const {
            Polygon convex_hull = spatial::convexHull(poly.getPoints());

            double poly_area = poly.area();
            double hull_area = convex_hull.area();

            if (hull_area < 1e-10)
                return 1.0; // Avoid division by zero
            return poly_area / hull_area;
        }

        // Calculate aspect ratio (length / width) from oriented bounding box
        double calculate_aspect_ratio(const Polygon &poly) const {
            // Use OBB to get aspect ratio
            Bound obb = poly.get_obb();

            double width = obb.size.x;
            double height = obb.size.y;

            if (std::min(width, height) < 1e-10)
                return 1.0; // Avoid division by zero
            return std::max(width, height) / std::min(width, height);
        }

        // Split polygon by area (simple geometric splitting)
        std::vector<Polygon> split_by_area(const Polygon &poly) const {
            double area = poly.area();
            if (area <= criteria_.max_area) {
                return {poly}; // No need to split
            }

            Bound obb = poly.get_obb();

            double width = obb.size.x;
            double height = obb.size.y;

            std::vector<Polygon> split_result;

            // Try splitting along both dimensions for better results
            // First try longer dimension
            if (width > height) {
                split_result = split_vertically(poly);

                // Check if the resulting parts are still too large
                bool needs_further_split = false;
                for (const auto &part : split_result) {
                    if (part.area() > criteria_.max_area) {
                        needs_further_split = true;
                        break;
                    }
                }

                // If parts are still too large, try another split
                if (needs_further_split && split_result.size() > 1) {
                    std::vector<Polygon> refined_result;
                    for (const auto &part : split_result) {
                        if (part.area() > criteria_.max_area) {
                            // Try splitting this part horizontally
                            auto sub_parts = split_horizontally(part);
                            refined_result.insert(refined_result.end(), sub_parts.begin(), sub_parts.end());
                        } else {
                            refined_result.push_back(part);
                        }
                    }
                    return refined_result;
                }
            } else {
                split_result = split_horizontally(poly);

                // Check if the resulting parts are still too large
                bool needs_further_split = false;
                for (const auto &part : split_result) {
                    if (part.area() > criteria_.max_area) {
                        needs_further_split = true;
                        break;
                    }
                }

                // If parts are still too large, try another split
                if (needs_further_split && split_result.size() > 1) {
                    std::vector<Polygon> refined_result;
                    for (const auto &part : split_result) {
                        if (part.area() > criteria_.max_area) {
                            // Try splitting this part vertically
                            auto sub_parts = split_vertically(part);
                            refined_result.insert(refined_result.end(), sub_parts.begin(), sub_parts.end());
                        } else {
                            refined_result.push_back(part);
                        }
                    }
                    return refined_result;
                }
            }

            return split_result;
        }

        // Helper method to split polygon vertically
        std::vector<Polygon> split_vertically(const Polygon &poly) const {
            const auto &points = poly.getPoints();
            if (points.size() < 3) {
                return {poly}; // Can't split
            }

            // Find bounding box
            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();

            for (const auto &p : points) {
                min_x = std::min(min_x, p.x);
                max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y);
                max_y = std::max(max_y, p.y);
            }

            double mid_x = (min_x + max_x) / 2.0;

            // Create cutting line with large but valid values (avoiding validation errors)
            double buffer = 1e6; // 1000 km buffer (large but still valid)
            Point p1{mid_x, min_y - buffer, 0};
            Point p2{mid_x, max_y + buffer, 0};
            Line cutting_line(p1, p2);

            return split_polygon_with_line(poly, cutting_line);
        }

        // Helper method to split polygon horizontally
        std::vector<Polygon> split_horizontally(const Polygon &poly) const {
            const auto &points = poly.getPoints();
            if (points.size() < 3) {
                return {poly}; // Can't split
            }

            // Find bounding box
            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();

            for (const auto &p : points) {
                min_x = std::min(min_x, p.x);
                max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y);
                max_y = std::max(max_y, p.y);
            }

            double mid_y = (min_y + max_y) / 2.0;

            // Create cutting line with large but valid values (avoiding validation errors)
            double buffer = 1e6; // 1000 km buffer (large but still valid)
            Point p1{min_x - buffer, mid_y, 0};
            Point p2{max_x + buffer, mid_y, 0};
            Line cutting_line(p1, p2);

            return split_polygon_with_line(poly, cutting_line);
        }

        // Detect and split narrow bridges using erosion and convex hull
        std::vector<Polygon> split_by_narrow_bridges(const Polygon &poly) const {
            // Use convex hull to find potential bridge locations
            Polygon convex_hull = spatial::convexHull(poly.getPoints());

            // Find concavities (potential bridge areas)
            std::vector<Line> potential_cuts;

            // Sample points on the convex hull and look for connections
            const auto &hull_points = convex_hull.getPoints();

            if (hull_points.size() < 3) {
                return {poly}; // Can't find bridges
            }

            // Look for pairs of points on the hull that could form bridge cuts
            for (size_t i = 0; i < hull_points.size(); i++) {
                for (size_t j = i + 2; j < hull_points.size(); j++) {
                    if (j - i == hull_points.size() - 1)
                        continue; // Skip adjacent points

                    const Point &p1 = hull_points[i];
                    const Point &p2 = hull_points[j];

                    // Line connecting these two hull points
                    Line potential_cut(p1, p2);

                    // Check if this line is mostly inside the polygon
                    if (is_potential_bridge_cut(poly, potential_cut)) {
                        potential_cuts.push_back(potential_cut);
                    }
                }
            }

            // Sort by length, shortest first (narrow bridges)
            std::sort(potential_cuts.begin(), potential_cuts.end(),
                      [](const Line &a, const Line &b) { return a.length() < b.length(); });

            // Try the shortest cuts first
            for (const auto &cut : potential_cuts) {
                auto split_result = split_polygon_with_line(poly, cut);
                if (split_result.size() > 1) {
                    return split_result;
                }
            }

            return {poly}; // No suitable cut found
        }

        // Helper function to check if a potential cut line might be a bridge
        bool is_potential_bridge_cut(const Polygon &poly, const Line &cut) const {
            // Sample points along the line and check if they're inside the polygon
            int num_samples = 10;
            int inside_count = 0;

            const Point &p1 = cut.getStart();
            const Point &p2 = cut.getEnd();

            for (int i = 0; i <= num_samples; i++) {
                double t = static_cast<double>(i) / num_samples;
                double x = p1.x + t * (p2.x - p1.x);
                double y = p1.y + t * (p2.y - p1.y);

                Point sample_point{x, y, 0.0};

                if (poly.contains(sample_point)) {
                    inside_count++;
                }
            }

            // Line should be mostly inside the polygon
            double inside_ratio = static_cast<double>(inside_count) / (num_samples + 1);
            return inside_ratio > 0.8 && cut.length() < criteria_.min_bridge_width * 3;
        }

        // Split polygon by detecting teeth and extensions
        std::vector<Polygon> split_by_teeth_and_extensions(const Polygon &poly) const {
            // Get convex hull
            Polygon convex_hull = spatial::convexHull(poly.getPoints());

            // Find significant concave regions (differences between hull and polygon)
            const auto &hull_points = convex_hull.getPoints();
            const auto &poly_points = poly.getPoints();

            // Look for potential cutting lines that would split off a "tooth"
            for (size_t i = 0; i < poly_points.size(); i++) {
                const Point &p1 = poly_points[i];
                const Point &p2 = poly_points[(i + 1) % poly_points.size()];

                // Skip edges that are on the convex hull
                bool is_hull_edge = false;
                for (size_t j = 0; j < hull_points.size(); j++) {
                    const Point &h1 = hull_points[j];
                    const Point &h2 = hull_points[(j + 1) % hull_points.size()];
                    if ((spatial::distance(p1, h1) < 1e-6 && spatial::distance(p2, h2) < 1e-6) ||
                        (spatial::distance(p1, h2) < 1e-6 && spatial::distance(p2, h1) < 1e-6)) {
                        is_hull_edge = true;
                        break;
                    }
                }

                if (!is_hull_edge) {
                    // This edge is part of a concavity
                    // Find a point on the hull to connect to
                    for (const auto &hull_point : hull_points) {
                        Line cut1(p1, hull_point);
                        Line cut2(p2, hull_point);

                        // Only consider cuts that stay within the polygon
                        if (is_valid_cutting_line(poly, cut1)) {
                            auto result = split_polygon_with_line(poly, cut1);
                            if (result.size() > 1) {
                                // Check if one part is small enough to be a "tooth"
                                if (is_tooth_like(result, poly.area())) {
                                    return result;
                                }
                            }
                        }

                        if (is_valid_cutting_line(poly, cut2)) {
                            auto result = split_polygon_with_line(poly, cut2);
                            if (result.size() > 1) {
                                if (is_tooth_like(result, poly.area())) {
                                    return result;
                                }
                            }
                        }
                    }
                }
            }

            return {poly}; // No suitable tooth cuts found
        }

        // Helper function to determine if a potential cut forms a valid cutting line
        bool is_valid_cutting_line(const Polygon &poly, const Line &cut) const {
            // Check if the line is mostly inside the polygon
            int num_samples = 10;
            int inside_count = 0;

            const Point &p1 = cut.getStart();
            const Point &p2 = cut.getEnd();

            for (int i = 1; i < num_samples; i++) { // Skip endpoints which are always on the boundary
                double t = static_cast<double>(i) / num_samples;
                double x = p1.x + t * (p2.x - p1.x);
                double y = p1.y + t * (p2.y - p1.y);

                Point sample_point{x, y, 0.0};

                if (poly.contains(sample_point)) {
                    inside_count++;
                }
            }

            // Most intermediate points should be inside
            return inside_count > (num_samples - 2) / 2;
        }

        // Helper function to check if the split results in a "tooth-like" subdivision
        bool is_tooth_like(const std::vector<Polygon> &parts, double original_area) const {
            if (parts.size() != 2)
                return false;

            double area1 = parts[0].area();
            double area2 = parts[1].area();

            double smaller_area = std::min(area1, area2);
            // We only care about the smaller part for tooth detection

            // Is the smaller part a significant tooth?
            return smaller_area < original_area * criteria_.tooth_threshold &&
                   smaller_area > original_area * 0.01; // Avoid tiny slivers
        }

        // Split elongated polygons by aspect ratio
        std::vector<Polygon> split_by_aspect_ratio(const Polygon &poly) const {
            Bound obb = poly.get_obb();

            double width = obb.size.x;
            double height = obb.size.y;

            if (width > height * criteria_.max_aspect_ratio) {
                // Too wide, split vertically
                return split_vertically(poly);
            } else if (height > width * criteria_.max_aspect_ratio) {
                // Too tall, split horizontally
                return split_horizontally(poly);
            }

            return {poly}; // Aspect ratio is acceptable
        }

        // Split highly concave polygons using convexity analysis
        std::vector<Polygon> split_by_convexity(const Polygon &poly) const {
            // Use built-in convex partition function
            PolygonList input = {poly};
            PolygonList output;

            TPPLPartition partitioner;
            partitioner.ConvexPartition_HM(&input, &output);

            std::vector<Polygon> result;

            if (output.size() > 1) {
                for (const auto &part : output) {
                    result.push_back(part);
                }
                return result;
            }

            // Fallback to basic area splitting if convex partition failed
            return split_by_area(poly);
        }

        // Split polygon into N approximately equal parts
        std::vector<Polygon> split_by_area_equal(const Polygon &poly, int num_parts) const {
            if (num_parts <= 1) {
                return {poly};
            }

            std::vector<Polygon> result;

            // For 2 parts, split along the longer dimension
            if (num_parts == 2) {
                auto bbox = AABB::fromPoints(poly.getPoints());
                double width = bbox.max_point.x - bbox.min_point.x;
                double height = bbox.max_point.y - bbox.min_point.y;

                Line split_line;
                if (width > height) {
                    // Split vertically at midpoint
                    double mid_x = (bbox.min_point.x + bbox.max_point.x) / 2.0;
                    split_line = Line(Point(mid_x, bbox.min_point.y - 100, 0), Point(mid_x, bbox.max_point.y + 100, 0));
                } else {
                    // Split horizontally at midpoint
                    double mid_y = (bbox.min_point.y + bbox.max_point.y) / 2.0;
                    split_line = Line(Point(bbox.min_point.x - 100, mid_y, 0), Point(bbox.max_point.x + 100, mid_y, 0));
                }

                return split_polygon_with_line(poly, split_line);
            }

            // For more parts, recursively split
            if (num_parts > 2) {
                // First split in half
                auto halves = split_by_area_equal(poly, 2);

                // Then recursively split each half
                for (const auto &half : halves) {
                    auto sub_parts = split_by_area_equal(half, num_parts / 2);
                    result.insert(result.end(), sub_parts.begin(), sub_parts.end());
                }

                return result;
            }

            return {poly};
        }

        // Balance areas of parts to be closer to target
        std::vector<Polygon> balance_part_areas(const std::vector<Polygon> &parts, double target_area,
                                                double tolerance) const {
            std::vector<Polygon> balanced;
            std::vector<std::pair<Polygon, double>> part_areas;

            // Calculate areas and store with polygons
            for (const auto &part : parts) {
                double area = part.area();
                if (area > target_area * 0.1) { // Filter out tiny parts
                    part_areas.push_back({part, area});
                }
            }

            // Sort by area descending
            std::sort(part_areas.begin(), part_areas.end(),
                      [](const auto &a, const auto &b) { return a.second > b.second; });

            // Process parts
            for (size_t i = 0; i < part_areas.size(); ++i) {
                const auto &[poly, area] = part_areas[i];

                // If part is within tolerance of target, keep it
                if (std::abs(area - target_area) / target_area <= tolerance) {
                    balanced.push_back(poly);
                }
                // If part is too large, try to split it
                else if (area > target_area * (1 + tolerance)) {
                    int subdivisions = std::round(area / target_area);
                    if (subdivisions > 1) {
                        // Use grid split for large parts
                        auto sub_parts = split_by_grid(poly, subdivisions);
                        for (const auto &sub : sub_parts) {
                            if (sub.area() > target_area * 0.1) {
                                balanced.push_back(sub);
                            }
                        }
                    } else {
                        balanced.push_back(poly);
                    }
                }
                // If part is too small, try to merge with adjacent small parts
                else {
                    balanced.push_back(poly); // For now, just keep it
                    // TODO: Implement merging logic for adjacent small parts
                }
            }

            // Report results
            std::cout << "Balanced " << parts.size() << " parts into " << balanced.size() << " parts" << std::endl;
            for (size_t i = 0; i < balanced.size(); ++i) {
                double area = balanced[i].area();
                std::cout << "  Part " << i << ": " << area << " sq.m (target: " << target_area
                          << ", deviation: " << std::abs(area - target_area) / target_area * 100 << "%)" << std::endl;
            }

            return balanced;
        }

        // Split polygon into an approximately grid-like pattern
        std::vector<Polygon> split_by_grid(const Polygon &poly, int divisions) const {
            if (divisions <= 1) {
                return {poly};
            }

            const auto &points = poly.getPoints();
            if (points.size() < 3) {
                return {poly}; // Can't split
            }

            // Find bounding box
            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();

            for (const auto &p : points) {
                min_x = std::min(min_x, p.x);
                max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y);
                max_y = std::max(max_y, p.y);
            }

            // Make sure we have enough divisions to get under the area threshold
            double total_area = poly.area();
            if (total_area > criteria_.max_area) {
                int min_divisions = std::ceil(std::sqrt(total_area / criteria_.max_area));
                divisions = std::max(divisions, min_divisions);
            }

            // Start with the original polygon
            std::vector<Polygon> result = {poly};
            std::vector<Polygon> intermediate;

            // First split horizontally multiple times
            double y_step = (max_y - min_y) / divisions;
            for (int i = 1; i < divisions; i++) {
                double cut_y = min_y + i * y_step;

                intermediate.clear();
                for (const auto &current_poly : result) {
                    // Create cutting line
                    double buffer = 1e6; // Large but valid buffer
                    Point p1{min_x - buffer, cut_y, 0};
                    Point p2{max_x + buffer, cut_y, 0};
                    Line cutting_line(p1, p2);

                    // Split this polygon
                    auto splits = split_polygon_with_line(current_poly, cutting_line);
                    intermediate.insert(intermediate.end(), splits.begin(), splits.end());
                }

                // Update result for next iteration
                if (!intermediate.empty()) {
                    result = intermediate;
                }
            }

            // Then split vertically multiple times
            double x_step = (max_x - min_x) / divisions;
            for (int i = 1; i < divisions; i++) {
                double cut_x = min_x + i * x_step;

                intermediate.clear();
                for (const auto &current_poly : result) {
                    // Create cutting line
                    double buffer = 1e6; // Large but valid buffer
                    Point p1{cut_x, min_y - buffer, 0};
                    Point p2{cut_x, max_y + buffer, 0};
                    Line cutting_line(p1, p2);

                    // Split this polygon
                    auto splits = split_polygon_with_line(current_poly, cutting_line);
                    intermediate.insert(intermediate.end(), splits.begin(), splits.end());
                }

                // Update result for next iteration
                if (!intermediate.empty()) {
                    result = intermediate;
                }
            }

            // Check if any resulting polygon is still too large
            std::vector<Polygon> final_result;
            for (const auto &part : result) {
                double part_area = part.area();
                if (part_area > criteria_.max_area) {
                    // Recursively split this part further with additional divisions
                    int local_divisions = std::ceil(std::sqrt(part_area / criteria_.max_area)) + 1;
                    auto sub_parts = split_by_grid(part, local_divisions);
                    final_result.insert(final_result.end(), sub_parts.begin(), sub_parts.end());
                } else {
                    final_result.push_back(part);
                }
            }

            return final_result;
        }

        // Helper function to split polygon with a line
        std::vector<Polygon> split_polygon_with_line(const Polygon &poly, const Line &cutting_line) const {
            // Implement polygon cutting using intersection calculations
            const auto &points = poly.getPoints();

            // Find intersections of the cutting line with polygon edges
            std::vector<Point> intersections;
            std::vector<size_t> edge_indices;

            for (size_t i = 0; i < points.size(); i++) {
                const Point &p1 = points[i];
                const Point &p2 = points[(i + 1) % points.size()];
                Line edge(p1, p2);

                Point intersection;
                if (spatial::lineIntersection(edge, cutting_line, intersection)) {
                    // Check if intersection is not at a vertex (avoid duplicates)
                    if (spatial::distance(intersection, p1) > 1e-6 && spatial::distance(intersection, p2) > 1e-6) {
                        intersections.push_back(intersection);
                        edge_indices.push_back(i);
                    }
                }
            }

            // Valid cut needs exactly 2 intersections
            if (intersections.size() != 2) {
                return {poly};
            }

            // Create two new polygons by splitting the original along the cutting line
            std::vector<Point> poly1_points, poly2_points;

            // First polygon contains first intersection, vertices until second intersection, then second intersection
            size_t idx1 = edge_indices[0];
            size_t idx2 = edge_indices[1];

            if (idx2 < idx1)
                std::swap(idx1, idx2);

            // Add first intersection point
            poly1_points.push_back(intersections[0]);

            // Add vertices between the intersection points
            for (size_t i = idx1 + 1; i <= idx2; i++) {
                poly1_points.push_back(points[i]);
            }

            // Add second intersection point
            poly1_points.push_back(intersections[1]);

            // Second polygon contains second intersection, remaining vertices, back to first intersection
            poly2_points.push_back(intersections[1]);

            // Add vertices after second intersection to end
            for (size_t i = idx2 + 1; i < points.size(); i++) {
                poly2_points.push_back(points[i]);
            }

            // Add vertices from start to first intersection
            for (size_t i = 0; i <= idx1; i++) {
                poly2_points.push_back(points[i]);
            }

            // Add first intersection point to close the loop
            poly2_points.push_back(intersections[0]);

            // Create the two new polygons
            std::vector<Polygon> result;

            // Only add polygons that have at least 3 vertices
            if (poly1_points.size() >= 3) {
                result.push_back(Polygon(poly1_points));
            }

            if (poly2_points.size() >= 3) {
                result.push_back(Polygon(poly2_points));
            }

            if (result.size() < 1) {
                // Failed to create valid polygons, return original
                return {poly};
            }

            return result;
        }
    };

} // namespace concord
