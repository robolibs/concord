#pragma once

#include "rtree/rtree.hpp"
#include "quadtree/quadtree.hpp" 
#include "hash_grid/spatial_hash_grid.hpp"
#include "../geometry/bounding.hpp"
#include "../geometry/polygon/polygon.hpp"

namespace concord {
    namespace indexing {

        /**
         * @brief Unified interface for spatial indexing operations
         * 
         * Provides consistent access patterns across different spatial index types
         * for common spatial querying operations.
         */
        namespace spatial_indexing {

            /**
             * @brief Point-based spatial queries using QuadTree
             * Optimal for: Point location queries, 2D range searches, k-NN on points
             */
            template<typename T>
            class PointIndex {
                QuadTree<T> tree_;
            public:
                PointIndex(const AABB& boundary) : tree_(boundary) {}
                
                bool insert(const Point& point, const T& data) { return tree_.insert(point, data); }
                bool remove(const Point& point, const T& data) { return tree_.remove(point, data); }
                
                std::vector<typename QuadTree<T>::Entry> queryRect(const AABB& rect) const {
                    return tree_.query(rect);
                }
                
                std::vector<typename QuadTree<T>::Entry> queryRadius(const Point& center, double radius) const {
                    return tree_.queryRadius(center, radius);
                }
                
                std::vector<typename QuadTree<T>::Entry> queryPolygon(const Polygon& polygon) const {
                    auto aabb = polygon.getAABB();
                    auto candidates = tree_.query(aabb);
                    std::vector<typename QuadTree<T>::Entry> results;
                    
                    for (const auto& candidate : candidates) {
                        if (polygon.contains(candidate.point)) {
                            results.push_back(candidate);
                        }
                    }
                    return results;
                }
                
                std::vector<typename QuadTree<T>::Entry> kNearestNeighbors(const Point& point, size_t k) const {
                    return tree_.kNearestNeighbors(point, k);
                }
                
                size_t size() const { return tree_.size(); }
                void clear() { tree_.clear(); }
            };

            /**
             * @brief Fast approximate spatial queries using SpatialHashGrid  
             * Optimal for: High-throughput point queries, real-time applications
             */
            template<typename T>
            class FastPointIndex {
                SpatialHashGrid<T> grid_;
            public:
                FastPointIndex(double cell_size) : grid_(cell_size) {}
                
                void insert(const Point& point, const T& data) { grid_.insert(point, data); }
                bool remove(const Point& point, const T& data) { return grid_.remove(point, data); }
                
                std::vector<T> queryRadius(const Point& center, double radius) const {
                    return grid_.query(center, radius);
                }
                
                std::vector<T> queryRect(const AABB& rect) const {
                    return grid_.queryRect(rect);
                }
                
                size_t size() const { return grid_.size(); }
                void clear() { grid_.clear(); }
                
                double getCellSize() const { return grid_.getCellSize(); }
                size_t getNumCells() const { return grid_.getNumCells(); }
            };

            /**
             * @brief Geometry-based spatial queries using RTree
             * Optimal for: Complex geometries, AABB queries, spatial objects with extent
             */
            template<typename T>
            class GeometryIndex {
                RTree<T> tree_;
            public:
                GeometryIndex(size_t max_entries = 16, size_t min_entries = 4) 
                    : tree_(max_entries, min_entries) {}
                
                void insert(const AABB& bounds, const T& data) { tree_.insert(bounds, data); }
                bool remove(const AABB& bounds, const T& data) { return tree_.remove(bounds, data); }
                
                std::vector<typename RTree<T>::Entry> queryRect(const AABB& rect) const {
                    return tree_.search(rect);
                }
                
                std::vector<typename RTree<T>::Entry> queryRadius(const Point& center, double radius) const {
                    return tree_.search_radius(center, radius);
                }
                
                std::vector<typename RTree<T>::Entry> queryPolygon(const Polygon& polygon) const {
                    auto aabb = polygon.getAABB();
                    auto candidates = tree_.search(aabb);
                    std::vector<typename RTree<T>::Entry> results;
                    
                    for (const auto& candidate : candidates) {
                        // Check if any corner of the candidate bounds is inside the polygon
                        auto corners = candidate.bounds.corners();
                        bool intersects = false;
                        for (const auto& corner : corners) {
                            if (polygon.contains(corner)) {
                                intersects = true;
                                break;
                            }
                        }
                        if (intersects) {
                            results.push_back(candidate);
                        }
                    }
                    return results;
                }
                
                std::vector<typename RTree<T>::Entry> kNearestNeighbors(const Point& point, size_t k) const {
                    return tree_.k_nearest_neighbors(point, k);
                }
                
                size_t size() const { return tree_.size(); }
                void clear() { tree_.clear(); }
            };

            /**
             * @brief Factory functions for creating spatial indices
             */
            template<typename T>
            PointIndex<T> createPointIndex(const AABB& boundary) {
                return PointIndex<T>(boundary);
            }

            template<typename T>
            FastPointIndex<T> createFastPointIndex(double cell_size) {
                return FastPointIndex<T>(cell_size);
            }

            template<typename T>
            GeometryIndex<T> createGeometryIndex(size_t max_entries = 16, size_t min_entries = 4) {
                return GeometryIndex<T>(max_entries, min_entries);
            }

            /**
             * @brief Utility functions for choosing appropriate index type
             */
            namespace utils {
                
                /**
                 * @brief Recommend optimal cell size for FastPointIndex based on data characteristics
                 */
                inline double recommendCellSize(const std::vector<Point>& points, double target_points_per_cell = 10.0) {
                    if (points.empty()) return 1.0;
                    
                    auto aabb = AABB::fromPoints(points);
                    double area = aabb.size().x * aabb.size().y;
                    double total_points = static_cast<double>(points.size());
                    
                    // Estimate cell size to achieve target density
                    double estimated_cells = total_points / target_points_per_cell;
                    double cell_area = area / estimated_cells;
                    return std::sqrt(cell_area);
                }
                
                /**
                 * @brief Check if point-based indexing is appropriate for the data
                 */
                template<typename Container>
                bool shouldUsePointIndex(const Container& geometries) {
                    // If all geometries are small relative to the space, point index is good
                    // This is a heuristic - in practice you'd analyze the actual geometry sizes
                    return geometries.size() > 100; // Simple heuristic
                }
            }

        } // namespace spatial_indexing

        // Convenience aliases in the main indexing namespace
        template<typename T> using PointIndex = spatial_indexing::PointIndex<T>;
        template<typename T> using FastPointIndex = spatial_indexing::FastPointIndex<T>;
        template<typename T> using GeometryIndex = spatial_indexing::GeometryIndex<T>;

    } // namespace indexing
} // namespace concord