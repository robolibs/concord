#pragma once

#include "../../geometry/bounding.hpp"
#include "../../types/point.hpp"
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <vector>

namespace concord {
    namespace indexing {

        // Spatial hash grid for fast approximate queries
        template <typename T> class SpatialHashGrid {
          private:
            struct Cell {
                std::vector<std::pair<Point, T>> items;
            };

            double cell_size_;
            std::unordered_map<int64_t, Cell> grid_;

            inline int64_t hash(int x, int y) const {
                // Simple hash combining x and y coordinates
                return (static_cast<int64_t>(x) << 32) | static_cast<int64_t>(y);
            }

            inline std::pair<int, int> getGridCoords(const Point &point) const {
                int x = static_cast<int>(std::floor(point.x / cell_size_));
                int y = static_cast<int>(std::floor(point.y / cell_size_));
                return {x, y};
            }

          public:
            SpatialHashGrid(double cell_size) : cell_size_(cell_size) {}

            inline void insert(const Point &point, const T &data) {
                auto [x, y] = getGridCoords(point);
                int64_t key = hash(x, y);
                grid_[key].items.emplace_back(point, data);
            }

            inline std::vector<T> query(const Point &center, double radius) const {
                std::vector<T> results;

                int min_x = static_cast<int>(std::floor((center.x - radius) / cell_size_));
                int max_x = static_cast<int>(std::floor((center.x + radius) / cell_size_));
                int min_y = static_cast<int>(std::floor((center.y - radius) / cell_size_));
                int max_y = static_cast<int>(std::floor((center.y + radius) / cell_size_));

                double radius_sq = radius * radius;

                for (int x = min_x; x <= max_x; ++x) {
                    for (int y = min_y; y <= max_y; ++y) {
                        int64_t key = hash(x, y);
                        auto it = grid_.find(key);
                        if (it != grid_.end()) {
                            for (const auto &[point, data] : it->second.items) {
                                double dx = point.x - center.x;
                                double dy = point.y - center.y;
                                if (dx * dx + dy * dy <= radius_sq) {
                                    results.push_back(data);
                                }
                            }
                        }
                    }
                }

                return results;
            }

            inline std::vector<T> queryRect(const AABB &rect) const {
                std::vector<T> results;

                int min_x = static_cast<int>(std::floor(rect.min_point.x / cell_size_));
                int max_x = static_cast<int>(std::floor(rect.max_point.x / cell_size_));
                int min_y = static_cast<int>(std::floor(rect.min_point.y / cell_size_));
                int max_y = static_cast<int>(std::floor(rect.max_point.y / cell_size_));

                for (int x = min_x; x <= max_x; ++x) {
                    for (int y = min_y; y <= max_y; ++y) {
                        int64_t key = hash(x, y);
                        auto it = grid_.find(key);
                        if (it != grid_.end()) {
                            for (const auto &[point, data] : it->second.items) {
                                if (rect.contains(point)) {
                                    results.push_back(data);
                                }
                            }
                        }
                    }
                }

                return results;
            }

            inline bool remove(const Point &point, const T &data) {
                auto [x, y] = getGridCoords(point);
                int64_t key = hash(x, y);
                auto it = grid_.find(key);
                if (it != grid_.end()) {
                    auto &items = it->second.items;
                    for (auto item_it = items.begin(); item_it != items.end(); ++item_it) {
                        if (item_it->first == point && item_it->second == data) {
                            items.erase(item_it);
                            if (items.empty()) {
                                grid_.erase(it);
                            }
                            return true;
                        }
                    }
                }
                return false;
            }

            inline void clear() { grid_.clear(); }

            inline size_t size() const {
                size_t total = 0;
                for (const auto &[key, cell] : grid_) {
                    total += cell.items.size();
                }
                return total;
            }

            inline double getCellSize() const { return cell_size_; }

            inline size_t getNumCells() const { return grid_.size(); }
        };

    } // namespace indexing
} // namespace concord