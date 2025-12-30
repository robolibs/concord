#pragma once

#include "graph.hpp"
#include <algorithm>
#include <graphix/vertex/algorithms/bfs.hpp>
#include <optional>
#include <queue>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace concord::transform {

    // ============================================================================
    // TypePairHash - Hash function for pair of type_index
    // ============================================================================
    struct TypePairHash {
        size_t operator()(const std::pair<std::type_index, std::type_index> &p) const {
            return std::hash<std::type_index>{}(p.first) ^ (std::hash<std::type_index>{}(p.second) << 1);
        }
    };

    // ============================================================================
    // TransformTree - High-level transform tree with path finding
    //
    // Wraps FrameGraph and provides:
    // - Path finding using BFS
    // - Transform composition along paths
    // - Caching of frequently used paths
    // ============================================================================
    class TransformTree {
      public:
        using VertexId = FrameGraph::VertexId;

        TransformTree() = default;

        // ========================================================================
        // Frame registration
        // ========================================================================

        // Register a frame by name
        template <typename Frame> inline VertexId register_frame(const std::string &name) {
            m_path_cache.clear(); // Invalidate cache when structure changes
            return m_graph.register_frame<Frame>(name);
        }

        // ========================================================================
        // Transform management
        // ========================================================================

        // Set transform between frames (by name)
        template <typename To, typename From, typename T = double>
        inline void set_transform(const std::string &to_frame, const std::string &from_frame,
                                  const concord::frame::Transform<To, From, T> &tf) {
            m_path_cache.clear(); // Invalidate cache when transforms change
            m_graph.set_transform(to_frame, from_frame, tf);
        }

        // ========================================================================
        // Transform lookup - the main feature
        // ========================================================================

        // Lookup transform between ANY two frames (chains through graph)
        // Returns a GenericTransform that can be used to transform points
        inline std::optional<GenericTransform> lookup(const std::string &to_frame,
                                                      const std::string &from_frame) const {
            // Same frame -> identity
            if (to_frame == from_frame) {
                if (!m_graph.has_frame(to_frame)) {
                    return std::nullopt;
                }
                return GenericTransform::identity();
            }

            // Get vertex IDs
            auto from_opt = m_graph.get_frame_id(from_frame);
            auto to_opt = m_graph.get_frame_id(to_frame);
            if (!from_opt || !to_opt) {
                return std::nullopt;
            }

            // Find path
            auto path = find_path(*from_opt, *to_opt);
            if (path.empty()) {
                return std::nullopt;
            }

            // Compose transforms along path
            return compose_path(path);
        }

        // Check if transform can be computed (path exists)
        inline bool can_transform(const std::string &to_frame, const std::string &from_frame) const {
            if (to_frame == from_frame) {
                return m_graph.has_frame(to_frame);
            }

            auto from_opt = m_graph.get_frame_id(from_frame);
            auto to_opt = m_graph.get_frame_id(to_frame);
            if (!from_opt || !to_opt) {
                return false;
            }

            auto path = find_path(*from_opt, *to_opt);
            return !path.empty();
        }

        // Get the path of frame names between two frames (for debugging)
        inline std::optional<std::vector<std::string>> get_path(const std::string &from_frame,
                                                                const std::string &to_frame) const {
            if (from_frame == to_frame) {
                if (m_graph.has_frame(from_frame)) {
                    return std::vector<std::string>{from_frame};
                }
                return std::nullopt;
            }

            auto from_opt = m_graph.get_frame_id(from_frame);
            auto to_opt = m_graph.get_frame_id(to_frame);
            if (!from_opt || !to_opt) {
                return std::nullopt;
            }

            auto path = find_path(*from_opt, *to_opt);
            if (path.empty()) {
                return std::nullopt;
            }

            std::vector<std::string> names;
            names.reserve(path.size());
            for (auto v : path) {
                names.push_back(m_graph.get_frame_name(v));
            }
            return names;
        }

        // ========================================================================
        // Typed lookup - for when you know the frame types at compile time
        // ========================================================================

        template <typename To, typename From, typename T = double>
        inline std::optional<concord::frame::Transform<To, From, T>> lookup() const {
            auto from_opt = m_graph.get_frame_id<From>();
            auto to_opt = m_graph.get_frame_id<To>();

            if (!from_opt || !to_opt) {
                return std::nullopt;
            }

            if (*from_opt == *to_opt) {
                return concord::frame::Transform<To, From, T>::identity();
            }

            auto path = find_path(*from_opt, *to_opt);
            if (path.empty()) {
                return std::nullopt;
            }

            auto generic = compose_path(path);
            if (!generic) {
                return std::nullopt;
            }

            // Convert GenericTransform to typed Transform
            return concord::frame::Transform<To, From, T>{concord::frame::Rotation<To, From, T>{generic->rotation},
                                                          generic->translation};
        }

        // Typed can_transform
        template <typename To, typename From> inline bool can_transform() const {
            auto from_opt = m_graph.get_frame_id<From>();
            auto to_opt = m_graph.get_frame_id<To>();
            if (!from_opt || !to_opt) {
                return false;
            }

            if (*from_opt == *to_opt) {
                return true;
            }

            auto path = find_path(*from_opt, *to_opt);
            return !path.empty();
        }

        // Typed get_path
        template <typename To, typename From> inline std::vector<std::string> get_path() const {
            auto from_opt = m_graph.get_frame_id<From>();
            auto to_opt = m_graph.get_frame_id<To>();

            if (!from_opt || !to_opt) {
                return {};
            }

            auto path = find_path(*from_opt, *to_opt);
            std::vector<std::string> names;
            names.reserve(path.size());
            for (auto v : path) {
                names.push_back(m_graph.get_frame_name(v));
            }
            return names;
        }

        // ========================================================================
        // Delegated methods from FrameGraph
        // ========================================================================

        inline bool has_frame(const std::string &name) const { return m_graph.has_frame(name); }

        template <typename Frame> inline bool has_frame() const { return m_graph.has_frame<Frame>(); }

        inline std::vector<std::string> frame_names() const { return m_graph.frame_names(); }

        inline size_t frame_count() const { return m_graph.frame_count(); }

        inline size_t transform_count() const { return m_graph.transform_count(); }

        inline void clear() {
            m_graph.clear();
            m_path_cache.clear();
        }

        // Access underlying graph (for advanced use)
        inline const FrameGraph &graph() const { return m_graph; }
        inline FrameGraph &graph() { return m_graph; }

      private:
        FrameGraph m_graph;

        // Edge key type (order-independent)
        using EdgeKey = std::pair<VertexId, VertexId>;

        static inline EdgeKey make_edge_key(VertexId a, VertexId b) { return (a < b) ? EdgeKey{a, b} : EdgeKey{b, a}; }

        // Hash for EdgeKey
        struct EdgeKeyHash {
            size_t operator()(const EdgeKey &key) const {
                return std::hash<VertexId>{}(key.first) ^ (std::hash<VertexId>{}(key.second) << 1);
            }
        };

        // Path cache: (from_vertex, to_vertex) -> vector of vertex IDs
        using PathCacheKey = std::pair<VertexId, VertexId>;
        mutable std::unordered_map<PathCacheKey, std::vector<VertexId>, EdgeKeyHash> m_path_cache;

        // ========================================================================
        // Path finding using BFS
        // ========================================================================

        inline std::vector<VertexId> find_path(VertexId from, VertexId to) const {
            if (from == to) {
                return {from};
            }

            // Check cache
            PathCacheKey cache_key{from, to};
            auto cache_it = m_path_cache.find(cache_key);
            if (cache_it != m_path_cache.end()) {
                return cache_it->second;
            }

            // Use graphix BFS
            auto bfs_result = graphix::vertex::algorithms::bfs(m_graph.graph(), from, to);

            std::vector<VertexId> path;
            if (bfs_result.target_found || bfs_result.distance.find(to) != bfs_result.distance.end()) {
                path = graphix::vertex::algorithms::reconstruct_path(bfs_result, from, to);
            }

            // Cache the result
            m_path_cache[cache_key] = path;
            return path;
        }

        // ========================================================================
        // Transform composition along a path
        // ========================================================================

        inline std::optional<GenericTransform> compose_path(const std::vector<VertexId> &path) const {
            if (path.empty()) {
                return std::nullopt;
            }

            if (path.size() == 1) {
                return GenericTransform::identity();
            }

            // Start with identity transform
            GenericTransform result = GenericTransform::identity();

            // Walk the path and compose transforms
            for (size_t i = 0; i < path.size() - 1; ++i) {
                VertexId current = path[i];
                VertexId next = path[i + 1];

                // Get the generic transform for this edge
                auto edge_tf_opt = m_graph.get_generic_transform(current, next);
                if (!edge_tf_opt) {
                    return std::nullopt; // Edge should exist if path was found
                }

                // Compose: result = edge_tf * result
                // For SE(3): T_A_B * T_B_C = T_A_C (transforms from C to A)
                // We want T_path[0]_path[n] = T_path[0]_path[1] * ... * T_path[n-1]_path[n]
                // Each edge_tf is T_next_current (transforms from current to next)
                // So we accumulate: edge_tf * result
                result = (*edge_tf_opt) * result;
            }

            return result;
        }
    };

} // namespace concord::transform
