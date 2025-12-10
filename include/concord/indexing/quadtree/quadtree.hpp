#pragma once

#include "../../geometry/bounding.hpp"
#include "../../types/point.hpp"
#include <algorithm>
#include <memory>
#include <vector>

namespace concord {
    namespace indexing {

        /**
         * @brief QuadTree spatial index implementation for 2D point data
         *
         * Efficient spatial data structure for 2D point queries with logarithmic
         * performance for insertion, removal, and range queries.
         *
         * @tparam T The type of data stored with each point
         */
        template <typename T> class QuadTree {
          public:
            struct Entry {
                Point point;
                T data;

                Entry(const Point &p, const T &d) : point(p), data(d) {}

                bool operator==(const Entry &other) const { return point == other.point && data == other.data; }

                bool operator!=(const Entry &other) const { return !(*this == other); }

                bool operator<(const Entry &other) const {
                    if (point.x != other.point.x)
                        return point.x < other.point.x;
                    if (point.y != other.point.y)
                        return point.y < other.point.y;
                    if (point.z != other.point.z)
                        return point.z < other.point.z;
                    return false; // For data comparison, we could add this if T supports it
                }
            };

          private:
            struct Node {
                AABB boundary;
                std::vector<Entry> entries;
                std::unique_ptr<Node> children[4]; // NW, NE, SW, SE
                static constexpr size_t CAPACITY = 16;

                Node(const AABB &bounds) : boundary(bounds) {}

                bool isLeaf() const { return children[0] == nullptr; }

                void subdivide() {
                    Point center = boundary.center();

                    // Create quadrants: NW, NE, SW, SE
                    children[0] = std::make_unique<Node>(
                        AABB{boundary.min_point, Point{center.x, boundary.max_point.y, boundary.max_point.z}});
                    children[1] = std::make_unique<Node>(
                        AABB{Point{center.x, center.y, boundary.min_point.z}, boundary.max_point});
                    children[2] = std::make_unique<Node>(
                        AABB{Point{boundary.min_point.x, boundary.min_point.y, boundary.min_point.z},
                             Point{center.x, center.y, boundary.max_point.z}});
                    children[3] =
                        std::make_unique<Node>(AABB{Point{center.x, boundary.min_point.y, boundary.min_point.z},
                                                    Point{boundary.max_point.x, center.y, boundary.max_point.z}});
                }
            };

            std::unique_ptr<Node> root_;

            bool insert(Node *node, const Entry &entry) {
                if (!node->boundary.contains(entry.point)) {
                    return false;
                }

                if (node->entries.size() < Node::CAPACITY && node->isLeaf()) {
                    node->entries.push_back(entry);
                    return true;
                }

                if (node->isLeaf()) {
                    node->subdivide();

                    // Redistribute existing entries
                    auto old_entries = std::move(node->entries);
                    node->entries.clear();

                    for (const auto &old_entry : old_entries) {
                        bool inserted = false;
                        for (int i = 0; i < 4; ++i) {
                            if (insert(node->children[i].get(), old_entry)) {
                                inserted = true;
                                break;
                            }
                        }
                        if (!inserted) {
                            node->entries.push_back(old_entry); // Keep in parent if doesn't fit children
                        }
                    }
                }

                // Try to insert in children
                for (int i = 0; i < 4; ++i) {
                    if (insert(node->children[i].get(), entry)) {
                        return true;
                    }
                }

                // If all children reject, keep in this node
                node->entries.push_back(entry);
                return true;
            }

            void query(Node *node, const AABB &range, std::vector<Entry> &results) const {
                if (!node || !node->boundary.intersects(range)) {
                    return;
                }

                for (const auto &entry : node->entries) {
                    if (range.contains(entry.point)) {
                        results.push_back(entry);
                    }
                }

                if (!node->isLeaf()) {
                    for (int i = 0; i < 4; ++i) {
                        query(node->children[i].get(), range, results);
                    }
                }
            }

            void queryRadius(Node *node, const Point &center, double radius, std::vector<Entry> &results) const {
                if (!node)
                    return;

                // Quick AABB check first
                AABB query_box{Point{center.x - radius, center.y - radius, center.z - radius},
                               Point{center.x + radius, center.y + radius, center.z + radius}};

                if (!node->boundary.intersects(query_box)) {
                    return;
                }

                double radius_sq = radius * radius;

                for (const auto &entry : node->entries) {
                    double dx = entry.point.x - center.x;
                    double dy = entry.point.y - center.y;
                    if (dx * dx + dy * dy <= radius_sq) {
                        results.push_back(entry);
                    }
                }

                if (!node->isLeaf()) {
                    for (int i = 0; i < 4; ++i) {
                        queryRadius(node->children[i].get(), center, radius, results);
                    }
                }
            }

            bool remove(Node *node, const Point &point, const T &data) {
                if (!node || !node->boundary.contains(point)) {
                    return false;
                }

                // Check entries in this node
                for (auto it = node->entries.begin(); it != node->entries.end(); ++it) {
                    if (it->point == point && it->data == data) {
                        node->entries.erase(it);
                        return true;
                    }
                }

                // Check children
                if (!node->isLeaf()) {
                    for (int i = 0; i < 4; ++i) {
                        if (remove(node->children[i].get(), point, data)) {
                            return true;
                        }
                    }
                }

                return false;
            }

            void kNearest(Node *node, const Point &point, size_t k,
                          std::vector<std::pair<double, Entry>> &candidates) const {
                if (!node)
                    return;

                // Add all entries from this node
                for (const auto &entry : node->entries) {
                    double dx = entry.point.x - point.x;
                    double dy = entry.point.y - point.y;
                    double distance_sq = dx * dx + dy * dy;
                    candidates.emplace_back(distance_sq, entry);
                }

                if (!node->isLeaf()) {
                    // Sort children by distance to query point
                    std::vector<std::pair<double, int>> child_distances;
                    for (int i = 0; i < 4; ++i) {
                        if (node->children[i]) {
                            double min_dist = node->children[i]->boundary.distance_to_point(point);
                            child_distances.emplace_back(min_dist * min_dist, i);
                        }
                    }

                    std::sort(child_distances.begin(), child_distances.end());

                    for (const auto &[dist, child_idx] : child_distances) {
                        kNearest(node->children[child_idx].get(), point, k, candidates);
                    }
                }
            }

            size_t countEntries(Node *node) const {
                if (!node)
                    return 0;

                size_t count = node->entries.size();
                if (!node->isLeaf()) {
                    for (int i = 0; i < 4; ++i) {
                        count += countEntries(node->children[i].get());
                    }
                }
                return count;
            }

          public:
            QuadTree(const AABB &boundary) : root_(std::make_unique<Node>(boundary)) {}

            /**
             * @brief Insert a point with associated data
             * @param point The point to insert
             * @param data The data to associate with the point
             * @return True if insertion was successful
             */
            bool insert(const Point &point, const T &data) {
                Entry entry(point, data);
                return insert(root_.get(), entry);
            }

            /**
             * @brief Query for all points within a rectangular region
             * @param range The AABB defining the query region
             * @return Vector of entries within the region
             */
            std::vector<Entry> query(const AABB &range) const {
                std::vector<Entry> results;
                query(root_.get(), range, results);
                return results;
            }

            /**
             * @brief Query for all points within a radius of a center point
             * @param center The center point for the circular query
             * @param radius The search radius
             * @return Vector of entries within the radius
             */
            std::vector<Entry> queryRadius(const Point &center, double radius) const {
                std::vector<Entry> results;
                queryRadius(root_.get(), center, radius, results);
                return results;
            }

            /**
             * @brief Remove a specific point and data combination
             * @param point The point to remove
             * @param data The data to remove
             * @return True if the entry was found and removed
             */
            bool remove(const Point &point, const T &data) { return remove(root_.get(), point, data); }

            /**
             * @brief Find k nearest neighbors to a point
             * @param point The query point
             * @param k Number of neighbors to find
             * @return Vector of k nearest entries (may be less than k if tree has fewer entries)
             */
            std::vector<Entry> kNearestNeighbors(const Point &point, size_t k) const {
                std::vector<std::pair<double, Entry>> candidates;
                kNearest(root_.get(), point, k, candidates);

                // Sort by distance and take first k
                std::sort(candidates.begin(), candidates.end());

                std::vector<Entry> results;
                size_t count = std::min(k, candidates.size());
                for (size_t i = 0; i < count; ++i) {
                    results.push_back(candidates[i].second);
                }
                return results;
            }

            /**
             * @brief Get the total number of entries in the QuadTree
             */
            size_t size() const { return countEntries(root_.get()); }

            /**
             * @brief Clear all entries from the QuadTree
             */
            void clear() { root_ = std::make_unique<Node>(root_->boundary); }

            /**
             * @brief Get the boundary of the QuadTree
             */
            const AABB &getBoundary() const { return root_->boundary; }
        };

    } // namespace indexing
} // namespace concord