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
            double max_area;
            double min_convexity;
            double max_aspect_ratio;
            double min_bridge_width;
            double tooth_threshold;
            double simplify_tolerance;
            int max_recursion_depth;
            bool enable_bridge_detection;
            bool enable_tooth_detection;
            bool enable_aspect_splitting;

            PartitionCriteria();
        };

      private:
        Polygon border_;
        PartitionCriteria criteria_;

      public:
        std::vector<Polygon> polygons_;

        Partitioner() = default;
        Partitioner(const Polygon &poly);

        std::vector<Polygon> partition(double area_threshold, const PartitionCriteria &criteria = PartitionCriteria{});
        std::vector<Polygon> partition_equal_areas(double target_area, double tolerance = 0.2);
        const PartitionCriteria &getCriteria() const;
        void setCriteria(const PartitionCriteria &criteria);

      private:
        void partition_recursive(const Polygon &poly, std::vector<Polygon> &result, int depth);
        bool meets_all_criteria(const Polygon &poly, double area) const;
        double calculate_convexity_ratio(const Polygon &poly) const;
        double calculate_aspect_ratio(const Polygon &poly) const;
        std::vector<Polygon> split_by_area(const Polygon &poly) const;
        std::vector<Polygon> split_by_narrow_bridges(const Polygon &poly) const;
        std::vector<Polygon> split_by_teeth(const Polygon &poly) const;
        std::vector<Polygon> split_by_elongation(const Polygon &poly) const;
        std::vector<Polygon> split_by_grid(const Polygon &poly, int divisions) const;
        std::vector<Polygon> split_by_area_equal(const Polygon &poly, int num_parts) const;
        std::vector<Polygon> balance_part_areas(const std::vector<Polygon> &parts, double target_area,
                                                double tolerance) const;
        bool should_split(const Polygon &poly) const;
        std::vector<Polygon> split_vertically(const Polygon &poly) const;
        std::vector<Polygon> split_horizontally(const Polygon &poly) const;
        std::vector<Polygon> split_polygon_with_line(const Polygon &poly, const Line &cut_line) const;
        std::vector<Polygon> split_by_teeth_and_extensions(const Polygon &poly) const;
        std::vector<Polygon> split_by_aspect_ratio(const Polygon &poly) const;
        std::vector<Polygon> split_by_convexity(const Polygon &poly) const;
        bool is_valid_cutting_line(const Polygon &poly, const Line &cut) const;
        bool is_tooth_like(const std::vector<Polygon> &parts, double original_area) const;
        bool is_potential_bridge_cut(const Polygon &poly, const Line &cut) const;
    };

} // namespace concord
