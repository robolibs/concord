#pragma once

#include "../../geometry/bounding.hpp"
#include "../../types/point.hpp"
#include <algorithm>
#include <memory>
#include <vector>

namespace concord {
    namespace indexing {

        /**
         * @brief R-tree spatial index implementation
         *
         * Hierarchical spatial data structure for efficient range queries,
         * nearest neighbor searches, and spatial operations.
         *
         * @tparam T The type of data stored in the R-tree
         */
        template <typename T> class RTree {
          public:
            struct Entry {
                AABB bounds;
                T data;

                Entry(const AABB &bounds, const T &data) : bounds(bounds), data(data) {}

                bool operator==(const Entry &other) const { return bounds == other.bounds && data == other.data; }

                bool operator!=(const Entry &other) const { return !(*this == other); }

                bool operator<(const Entry &other) const {
                    if (bounds.min_point.x != other.bounds.min_point.x)
                        return bounds.min_point.x < other.bounds.min_point.x;
                    if (bounds.min_point.y != other.bounds.min_point.y)
                        return bounds.min_point.y < other.bounds.min_point.y;
                    if (bounds.min_point.z != other.bounds.min_point.z)
                        return bounds.min_point.z < other.bounds.min_point.z;
                    return false; // For data comparison, we could add this if T supports it
                }
            };

            struct Node {
                AABB bounds;
                std::vector<Entry> entries;
                std::vector<std::shared_ptr<Node>> children;
                bool is_leaf = true;

                Node() = default;
            };

          private:
            std::shared_ptr<Node> root_;
            size_t max_entries_ = 4; // M parameter
            size_t min_entries_ = 2; // m parameter

          public:
            /**
             * @brief Construct R-tree with specified parameters
             * @param max_entries Maximum entries per node (M)
             * @param min_entries Minimum entries per node (m)
             */
            RTree(size_t max_entries = 4, size_t min_entries = 2)
                : max_entries_(max_entries), min_entries_(min_entries) {
                root_ = std::make_shared<Node>();
            }

            /**
             * @brief Insert an entry into the R-tree
             * @param bounds Bounding box of the entry
             * @param data Data associated with the entry
             */
            void insert(const AABB &bounds, const T &data) {
                Entry entry(bounds, data);
                insert_entry(root_, entry);
            }

            /**
             * @brief Search for entries that intersect with the given bounds
             * @param query_bounds The bounding box to search within
             * @return Vector of entries that intersect with query_bounds
             */
            std::vector<Entry> search(const AABB &query_bounds) const {
                std::vector<Entry> results;
                search_recursive(root_, query_bounds, results);
                return results;
            }

            /**
             * @brief Find all entries within a given distance from a point
             * @param point The query point
             * @param radius The search radius
             * @return Vector of entries within the radius
             */
            std::vector<Entry> search_radius(const Point &point, double radius) const {
                AABB query_bounds(Point(point.x - radius, point.y - radius, point.z - radius),
                                  Point(point.x + radius, point.y + radius, point.z + radius));

                auto candidates = search(query_bounds);
                std::vector<Entry> results;

                for (const auto &entry : candidates) {
                    // Check actual distance to entry bounds
                    if (entry.bounds.distance_to_point(point) <= radius) {
                        results.push_back(entry);
                    }
                }

                return results;
            }

            /**
             * @brief Get the total number of entries in the R-tree
             */
            size_t size() const { return count_entries(root_); }

            /**
             * @brief Remove an entry from the R-tree
             * @param bounds Bounding box of the entry to remove
             * @param data Data associated with the entry to remove
             * @return True if entry was found and removed, false otherwise
             */
            bool remove(const AABB &bounds, const T &data) {
                Entry entry(bounds, data);
                return remove_entry(root_, entry);
            }

            /**
             * @brief Find k nearest neighbors to a point
             * @param point The query point
             * @param k Number of neighbors to find
             * @return Vector of k nearest entries (may be less than k if tree has fewer entries)
             */
            std::vector<Entry> k_nearest_neighbors(const Point &point, size_t k) const {
                std::vector<std::pair<double, Entry>> candidates;
                k_nearest_recursive(root_, point, candidates);

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
             * @brief Clear all entries from the R-tree
             */
            void clear() { root_ = std::make_shared<Node>(); }

          private:
            std::shared_ptr<Node> insert_entry(std::shared_ptr<Node> node, const Entry &entry) {
                if (node->is_leaf) {
                    node->entries.push_back(entry);
                    update_bounds(node);

                    if (node->entries.size() > max_entries_) {
                        return split_node(node);
                    }
                    return nullptr;
                } else {
                    // Find the child node with minimum area increase
                    auto best_child = choose_subtree(node, entry.bounds);
                    auto new_child = insert_entry(best_child, entry);

                    if (new_child) {
                        // Child was split, add the new child to this node
                        node->children.push_back(new_child);
                        update_bounds(node);

                        if (node->children.size() > max_entries_) {
                            return split_node(node);
                        }
                    } else {
                        update_bounds(node);
                    }
                    return nullptr;
                }
            }

            std::shared_ptr<Node> choose_subtree(std::shared_ptr<Node> node, const AABB &bounds) {
                double min_enlargement = std::numeric_limits<double>::max();
                double min_area = std::numeric_limits<double>::max();
                std::shared_ptr<Node> best_child = nullptr;

                for (auto &child : node->children) {
                    double current_area = child->bounds.area();
                    double enlargement = child->bounds.union_with(bounds).area() - current_area;

                    // R*-tree: prefer minimum enlargement, then minimum area
                    if (enlargement < min_enlargement || (enlargement == min_enlargement && current_area < min_area)) {
                        min_enlargement = enlargement;
                        min_area = current_area;
                        best_child = child;
                    }
                }

                return best_child;
            }

            std::shared_ptr<Node> split_node(std::shared_ptr<Node> node) {
                // R*-tree quadratic split algorithm
                auto new_node = std::make_shared<Node>();
                new_node->is_leaf = node->is_leaf;

                if (node->is_leaf) {
                    split_entries_quadratic(node, new_node);
                } else {
                    split_children_quadratic(node, new_node);
                }

                update_bounds(node);
                update_bounds(new_node);

                // If this was the root, create a new root
                if (node.get() == root_.get()) {
                    auto new_root = std::make_shared<Node>();
                    new_root->is_leaf = false;
                    new_root->children.push_back(node);
                    new_root->children.push_back(new_node);
                    update_bounds(new_root);
                    root_ = new_root;
                    return nullptr; // Root split handled internally
                }

                // For non-root nodes, return the new node to be added to parent
                return new_node;
            }

            void update_bounds(std::shared_ptr<Node> node) {
                if (node->entries.empty() && node->children.empty()) {
                    return;
                }

                AABB bounds;
                bool first = true;

                for (const auto &entry : node->entries) {
                    if (first) {
                        bounds = entry.bounds;
                        first = false;
                    } else {
                        bounds = bounds.union_with(entry.bounds);
                    }
                }

                for (const auto &child : node->children) {
                    if (first) {
                        bounds = child->bounds;
                        first = false;
                    } else {
                        bounds = bounds.union_with(child->bounds);
                    }
                }

                node->bounds = bounds;
            }

            void search_recursive(std::shared_ptr<Node> node, const AABB &query_bounds,
                                  std::vector<Entry> &results) const {
                if (!node->bounds.intersects(query_bounds)) {
                    return;
                }

                if (node->is_leaf) {
                    for (const auto &entry : node->entries) {
                        if (entry.bounds.intersects(query_bounds)) {
                            results.push_back(entry);
                        }
                    }
                } else {
                    for (const auto &child : node->children) {
                        search_recursive(child, query_bounds, results);
                    }
                }
            }

            size_t count_entries(std::shared_ptr<Node> node) const {
                if (node->is_leaf) {
                    return node->entries.size();
                }

                size_t count = 0;
                for (const auto &child : node->children) {
                    count += count_entries(child);
                }
                return count;
            }

            void split_entries_quadratic(std::shared_ptr<Node> node, std::shared_ptr<Node> new_node) {
                auto &entries = node->entries;

                // Find the two entries with maximum separation
                size_t seed1 = 0, seed2 = 1;
                double max_waste = -1;

                for (size_t i = 0; i < entries.size(); ++i) {
                    for (size_t j = i + 1; j < entries.size(); ++j) {
                        AABB combined = entries[i].bounds.union_with(entries[j].bounds);
                        double waste = combined.area() - entries[i].bounds.area() - entries[j].bounds.area();
                        if (waste > max_waste) {
                            max_waste = waste;
                            seed1 = i;
                            seed2 = j;
                        }
                    }
                }

                // Store seeds before modifying the vector
                Entry seed1_entry = entries[seed1];
                Entry seed2_entry = entries[seed2];

                // Remove seeds from entries (remove higher index first to avoid index shift)
                if (seed1 > seed2) {
                    entries.erase(entries.begin() + seed1);
                    entries.erase(entries.begin() + seed2);
                } else {
                    entries.erase(entries.begin() + seed2);
                    entries.erase(entries.begin() + seed1);
                }

                // Distribute remaining entries
                std::vector<Entry> remaining = entries;

                // Clear original entries and start fresh with seed1
                entries.clear();
                entries.push_back(seed1_entry);

                // New node starts with seed2
                new_node->entries.push_back(seed2_entry);

                for (const auto &entry : remaining) {
                    // Calculate cost of adding to each group
                    AABB node_bounds = calculate_bounds_entries(node->entries);
                    AABB new_node_bounds = calculate_bounds_entries(new_node->entries);

                    double cost1 = node_bounds.union_with(entry.bounds).area() - node_bounds.area();
                    double cost2 = new_node_bounds.union_with(entry.bounds).area() - new_node_bounds.area();

                    if (cost1 < cost2 || (cost1 == cost2 && node->entries.size() < new_node->entries.size())) {
                        node->entries.push_back(entry);
                    } else {
                        new_node->entries.push_back(entry);
                    }
                }
            }

            void split_children_quadratic(std::shared_ptr<Node> node, std::shared_ptr<Node> new_node) {
                auto &children = node->children;

                // Find the two children with maximum separation
                size_t seed1 = 0, seed2 = 1;
                double max_waste = -1;

                for (size_t i = 0; i < children.size(); ++i) {
                    for (size_t j = i + 1; j < children.size(); ++j) {
                        AABB combined = children[i]->bounds.union_with(children[j]->bounds);
                        double waste = combined.area() - children[i]->bounds.area() - children[j]->bounds.area();
                        if (waste > max_waste) {
                            max_waste = waste;
                            seed1 = i;
                            seed2 = j;
                        }
                    }
                }

                // Store seeds before modifying the vector
                auto seed1_child = children[seed1];
                auto seed2_child = children[seed2];

                // Remove seeds from children (remove higher index first to avoid index shift)
                if (seed1 > seed2) {
                    children.erase(children.begin() + seed1);
                    children.erase(children.begin() + seed2);
                } else {
                    children.erase(children.begin() + seed2);
                    children.erase(children.begin() + seed1);
                }

                // Distribute remaining children
                std::vector<std::shared_ptr<Node>> remaining = children;

                // Clear original children and start fresh with seed1
                children.clear();
                children.push_back(seed1_child);

                // New node starts with seed2
                new_node->children.push_back(seed2_child);

                for (const auto &child : remaining) {
                    // Calculate cost of adding to each group
                    AABB node_bounds = calculate_bounds_children(node->children);
                    AABB new_node_bounds = calculate_bounds_children(new_node->children);

                    double cost1 = node_bounds.union_with(child->bounds).area() - node_bounds.area();
                    double cost2 = new_node_bounds.union_with(child->bounds).area() - new_node_bounds.area();

                    if (cost1 < cost2 || (cost1 == cost2 && node->children.size() < new_node->children.size())) {
                        node->children.push_back(child);
                    } else {
                        new_node->children.push_back(child);
                    }
                }
            }

            AABB calculate_bounds_entries(const std::vector<Entry> &entries) const {
                if (entries.empty()) {
                    return AABB();
                }

                AABB bounds = entries[0].bounds;
                for (size_t i = 1; i < entries.size(); ++i) {
                    bounds = bounds.union_with(entries[i].bounds);
                }
                return bounds;
            }

            AABB calculate_bounds_children(const std::vector<std::shared_ptr<Node>> &children) const {
                if (children.empty()) {
                    return AABB();
                }

                AABB bounds = children[0]->bounds;
                for (size_t i = 1; i < children.size(); ++i) {
                    bounds = bounds.union_with(children[i]->bounds);
                }
                return bounds;
            }

            bool remove_entry(std::shared_ptr<Node> node, const Entry &entry) {
                if (node->is_leaf) {
                    // Search for entry in leaf
                    for (auto it = node->entries.begin(); it != node->entries.end(); ++it) {
                        if (it->bounds == entry.bounds && it->data == entry.data) {
                            node->entries.erase(it);
                            update_bounds(node);
                            return true;
                        }
                    }
                    return false;
                } else {
                    // Search in children
                    for (auto &child : node->children) {
                        if (child->bounds.intersects(entry.bounds)) {
                            if (remove_entry(child, entry)) {
                                update_bounds(node);
                                // TODO: Handle underflow and tree restructuring
                                return true;
                            }
                        }
                    }
                    return false;
                }
            }

            void k_nearest_recursive(std::shared_ptr<Node> node, const Point &point,
                                     std::vector<std::pair<double, Entry>> &candidates) const {
                if (!node)
                    return;

                if (node->is_leaf) {
                    // Add all entries with their distances
                    for (const auto &entry : node->entries) {
                        double distance = entry.bounds.distance_to_point(point);
                        candidates.emplace_back(distance, entry);
                    }
                } else {
                    // Process children ordered by their minimum distance to point
                    std::vector<std::pair<double, std::shared_ptr<Node>>> child_distances;
                    for (const auto &child : node->children) {
                        double min_distance = child->bounds.distance_to_point(point);
                        child_distances.emplace_back(min_distance, child);
                    }

                    std::sort(child_distances.begin(), child_distances.end());

                    for (const auto &[distance, child] : child_distances) {
                        k_nearest_recursive(child, point, candidates);
                    }
                }
            }
        };

    } // namespace indexing
} // namespace concord
