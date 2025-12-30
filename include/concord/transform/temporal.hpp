#pragma once

#include "graph.hpp"
#include "tree.hpp"
#include <algorithm>
#include <cmath>
#include <datapod/datapod.hpp>
#include <datapod/temporal/stamp.hpp>
#include <datapod/temporal/time_series.hpp>
#include <graphix/vertex/algorithms/bfs.hpp>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace concord::transform {

    // Namespace alias for datapod (exported so users of this header can use dp::)
    namespace dp = ::datapod;

} // namespace concord::transform

// Also export dp:: to global scope for convenience in user code
namespace dp = ::datapod;

namespace concord::transform {

    // ============================================================================
    // Transform Interpolation Functions
    // ============================================================================

    /**
     * @brief Spherical linear interpolation (SLERP) for quaternions
     *
     * This is a wrapper around datapod's slerp to provide a consistent interface
     * within the concord::transform namespace.
     *
     * @param q0 Start quaternion
     * @param q1 End quaternion
     * @param t Interpolation factor [0, 1]
     * @return Interpolated quaternion
     */
    inline dp::Quaternion slerp(const dp::Quaternion &q0, const dp::Quaternion &q1, double t) {
        // Use datapod's slerp implementation for Quaternion
        return ::datapod::slerp(q0, q1, t);
    }

    /**
     * @brief Linear interpolation for translation (dp::Point)
     * @param p0 Start point
     * @param p1 End point
     * @param t Interpolation factor [0, 1]
     * @return Interpolated point
     */
    inline dp::Point lerp(const dp::Point &p0, const dp::Point &p1, double t) {
        return dp::Point{p0.x + t * (p1.x - p0.x), p0.y + t * (p1.y - p0.y), p0.z + t * (p1.z - p0.z)};
    }

    /**
     * @brief Interpolate between two GenericTransforms
     * @param tf0 Start transform
     * @param tf1 End transform
     * @param t Interpolation factor [0, 1]
     * @return Interpolated transform
     */
    inline GenericTransform interpolate(const GenericTransform &tf0, const GenericTransform &tf1, double t) {
        dp::Quaternion interp_rot = ::datapod::slerp(tf0.rotation, tf1.rotation, t);
        dp::Point interp_trans = lerp(tf0.translation, tf1.translation, t);
        return GenericTransform(interp_rot, interp_trans);
    }

    // ============================================================================
    // TimedTransformBuffer - Stores transform history for a single edge
    // ============================================================================

    /**
     * @brief Circular buffer storing timestamped transforms for a single edge
     *
     * Provides interpolation between stored transforms for querying at
     * arbitrary timestamps within the buffered time range.
     *
     * @tparam BufferSize Maximum number of transforms to store
     */
    template <size_t BufferSize = 100> class TimedTransformBuffer {
      public:
        TimedTransformBuffer() = default;

        /**
         * @brief Construct with custom max size
         * @param max_size Maximum number of transforms to store
         */
        explicit TimedTransformBuffer(size_t max_size) : m_max_size(max_size) {}

        /**
         * @brief Add a timestamped transform (double timestamp)
         * @param timestamp Time in seconds (as double)
         * @param tf Transform to store
         */
        inline void add(double timestamp, const GenericTransform &tf) {
            m_timestamps.push_back(timestamp);
            m_values.push_back(tf);

            // Enforce max size by removing oldest entries
            while (m_timestamps.size() > m_max_size) {
                m_timestamps.erase(m_timestamps.begin());
                m_values.erase(m_values.begin());
            }

            // Ensure sorted order by timestamp
            sort_by_time();
        }

        /**
         * @brief Add a timestamped transform (int64 timestamp)
         * @param timestamp Time in nanoseconds
         * @param tf Transform to store
         */
        inline void add(int64_t timestamp, const GenericTransform &tf) {
            add(static_cast<double>(timestamp) / 1e9, tf);
        }

        /**
         * @brief Add a stamped transform
         * @param stamped Timestamped transform
         */
        inline void add(const dp::Stamp<GenericTransform> &stamped) {
            add(static_cast<double>(stamped.timestamp) / 1e9, stamped.value);
        }

        /**
         * @brief Get transform at specific time with interpolation (double timestamp)
         * @param timestamp Time in seconds
         * @return Interpolated transform, or nullopt if out of range
         */
        inline std::optional<GenericTransform> lookup(double timestamp) const {
            if (m_timestamps.empty()) {
                return std::nullopt;
            }

            // Single element
            if (m_timestamps.size() == 1) {
                // Exact match only for single element
                if (std::abs(m_timestamps[0] - timestamp) < 1e-10) {
                    return std::optional<GenericTransform>(m_values[0]);
                }
                return std::nullopt;
            }

            // Check if timestamp is within range
            if (!in_range(timestamp)) {
                return std::nullopt;
            }

            // Find bounding transforms using binary search
            auto it = std::lower_bound(m_timestamps.begin(), m_timestamps.end(), timestamp);

            // Exact match
            if (it != m_timestamps.end() && std::abs(*it - timestamp) < 1e-10) {
                size_t idx = static_cast<size_t>(it - m_timestamps.begin());
                return std::optional<GenericTransform>(m_values[idx]);
            }

            // Need to interpolate
            if (it == m_timestamps.begin()) {
                // Before first timestamp (shouldn't happen due to in_range check)
                return std::optional<GenericTransform>(m_values[0]);
            }

            if (it == m_timestamps.end()) {
                // After last timestamp (shouldn't happen due to in_range check)
                return std::optional<GenericTransform>(m_values.back());
            }

            // Interpolate between previous and current
            size_t idx_after = static_cast<size_t>(it - m_timestamps.begin());
            size_t idx_before = idx_after - 1;

            double t0 = m_timestamps[idx_before];
            double t1 = m_timestamps[idx_after];

            // Calculate interpolation factor
            double interp_t = (timestamp - t0) / (t1 - t0);

            return std::optional<GenericTransform>(interpolate(m_values[idx_before], m_values[idx_after], interp_t));
        }

        /**
         * @brief Get transform at specific time with interpolation (int64 timestamp)
         * @param timestamp Time in nanoseconds
         * @return Interpolated transform, or nullopt if out of range
         */
        inline std::optional<GenericTransform> get(int64_t timestamp) const {
            return lookup(static_cast<double>(timestamp) / 1e9);
        }

        /**
         * @brief Get the latest (most recent) transform
         * @return Latest transform, or nullopt if empty
         */
        inline std::optional<GenericTransform> latest() const {
            if (m_timestamps.empty()) {
                return std::nullopt;
            }
            return std::optional<GenericTransform>(m_values.back());
        }

        /**
         * @brief Get the time range covered by this buffer
         * @return Optional pair of (oldest_timestamp, newest_timestamp) in seconds
         */
        inline std::optional<std::pair<double, double>> time_range() const {
            if (m_timestamps.empty()) {
                return std::nullopt;
            }
            return std::make_pair(m_timestamps.front(), m_timestamps.back());
        }

        /**
         * @brief Check if timestamp (in seconds) is within the buffered range
         * @param timestamp Time in seconds
         * @return true if timestamp is within [oldest, newest]
         */
        inline bool in_range(double timestamp) const {
            if (m_timestamps.empty()) {
                return false;
            }
            return timestamp >= m_timestamps.front() && timestamp <= m_timestamps.back();
        }

        /**
         * @brief Check if timestamp (in nanoseconds) is within the buffered range
         * @param timestamp Time in nanoseconds
         * @return true if timestamp is within [oldest, newest]
         */
        inline bool in_range(int64_t timestamp) const { return in_range(static_cast<double>(timestamp) / 1e9); }

        /**
         * @brief Get number of stored transforms
         * @return Number of transforms in buffer
         */
        inline size_t size() const { return m_timestamps.size(); }

        /**
         * @brief Check if buffer is empty
         * @return true if no transforms stored
         */
        inline bool empty() const { return m_timestamps.empty(); }

        /**
         * @brief Clear all stored transforms
         */
        inline void clear() {
            m_timestamps.clear();
            m_values.clear();
        }

      private:
        std::vector<double> m_timestamps;
        std::vector<GenericTransform> m_values;
        size_t m_max_size = BufferSize;

        inline void sort_by_time() {
            if (m_timestamps.size() <= 1) {
                return;
            }

            // Create index array
            std::vector<size_t> indices(m_timestamps.size());
            for (size_t i = 0; i < m_timestamps.size(); ++i) {
                indices[i] = i;
            }

            // Sort indices by timestamp
            std::sort(indices.begin(), indices.end(),
                      [this](size_t a, size_t b) { return m_timestamps[a] < m_timestamps[b]; });

            // Reorder both arrays
            std::vector<double> sorted_times(m_timestamps.size());
            std::vector<GenericTransform> sorted_values(m_values.size());

            for (size_t i = 0; i < m_timestamps.size(); ++i) {
                sorted_times[i] = m_timestamps[indices[i]];
                sorted_values[i] = m_values[indices[i]];
            }

            m_timestamps = std::move(sorted_times);
            m_values = std::move(sorted_values);
        }
    };

    // ============================================================================
    // TimedTransformTree - Transform tree with temporal support
    // ============================================================================

    /**
     * @brief Transform tree supporting time-interpolated lookups
     *
     * Extends the basic TransformTree concept with:
     * - Timestamped transform storage
     * - Interpolation between stored transforms
     * - Static transforms (constant, no time history)
     * - Path composition at specific timestamps
     */
    class TimedTransformTree {
      public:
        using VertexId = FrameGraph::VertexId;

        TimedTransformTree() = default;

        // ========================================================================
        // Frame Registration
        // ========================================================================

        /**
         * @brief Register a frame with the tree
         * @tparam Frame Frame tag type
         * @param name Human-readable frame name
         * @return Vertex ID for the frame
         */
        template <typename Frame> inline VertexId register_frame(const std::string &name) {
            m_path_cache.clear();
            return m_graph.register_frame<Frame>(name);
        }

        // ========================================================================
        // Dynamic Transform Management
        // ========================================================================

        /**
         * @brief Set a transform at a specific time (double timestamp)
         * @tparam To Target frame type
         * @tparam From Source frame type
         * @tparam T Scalar type (default: double)
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param tf Transform to set
         * @param timestamp Time in seconds
         */
        template <typename To, typename From, typename T = double>
        inline void set_transform(const std::string &to_frame, const std::string &from_frame,
                                  const concord::frame::Transform<To, From, T> &tf, double timestamp) {
            auto from_opt = m_graph.get_frame_id(from_frame);
            auto to_opt = m_graph.get_frame_id(to_frame);

            if (!from_opt || !to_opt) {
                return;
            }

            EdgeKey edge_key = make_edge_key(*from_opt, *to_opt);

            // Remove from static if it was static
            m_static_edges.erase(edge_key);
            m_static_transforms.erase(edge_key);

            // Add to dynamic buffer
            GenericTransform generic(tf.rotation.quaternion(), tf.translation);

            // Store direction info
            m_edge_directions[edge_key] = {*from_opt, *to_opt};

            m_dynamic_transforms[edge_key].add(timestamp, generic);

            // Ensure edge exists in graph
            if (!m_graph.has_edge(*from_opt, *to_opt)) {
                m_graph.graph().add_edge(*from_opt, *to_opt, 1.0, graphix::vertex::EdgeType::Undirected);
            }

            m_path_cache.clear();
        }

        /**
         * @brief Set a transform at a specific time (int64 timestamp)
         * @tparam To Target frame type
         * @tparam From Source frame type
         * @tparam T Scalar type (default: double)
         * @param timestamp Time in nanoseconds
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param tf Transform to set
         */
        template <typename To, typename From, typename T = double>
        inline void set(int64_t timestamp, const std::string &to_frame, const std::string &from_frame,
                        const concord::frame::Transform<To, From, T> &tf) {
            set_transform<To, From, T>(to_frame, from_frame, tf, static_cast<double>(timestamp) / 1e9);
        }

        /**
         * @brief Set a transform using a Stamp
         * @tparam To Target frame type
         * @tparam From Source frame type
         * @tparam T Scalar type (default: double)
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param stamped Timestamped transform
         */
        template <typename To, typename From, typename T = double>
        inline void set(const std::string &to_frame, const std::string &from_frame,
                        const dp::Stamp<concord::frame::Transform<To, From, T>> &stamped) {
            set_transform<To, From, T>(to_frame, from_frame, stamped.value,
                                       static_cast<double>(stamped.timestamp) / 1e9);
        }

        // ========================================================================
        // Static Transform Management
        // ========================================================================

        /**
         * @brief Set a static (time-invariant) transform
         * @tparam To Target frame type
         * @tparam From Source frame type
         * @tparam T Scalar type (default: double)
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param tf Transform to set
         */
        template <typename To, typename From, typename T = double>
        inline void set_static_transform(const std::string &to_frame, const std::string &from_frame,
                                         const concord::frame::Transform<To, From, T> &tf) {
            auto from_opt = m_graph.get_frame_id(from_frame);
            auto to_opt = m_graph.get_frame_id(to_frame);

            if (!from_opt || !to_opt) {
                return;
            }

            EdgeKey edge_key = make_edge_key(*from_opt, *to_opt);

            // Remove from dynamic if it was dynamic
            m_dynamic_transforms.erase(edge_key);

            // Add to static
            GenericTransform generic(tf.rotation.quaternion(), tf.translation);
            m_static_transforms[edge_key] = generic;
            m_static_edges.insert(edge_key);

            // Store direction info
            m_edge_directions[edge_key] = {*from_opt, *to_opt};

            // Ensure edge exists in graph
            if (!m_graph.has_edge(*from_opt, *to_opt)) {
                m_graph.graph().add_edge(*from_opt, *to_opt, 1.0, graphix::vertex::EdgeType::Undirected);
            }

            m_path_cache.clear();
        }

        // Alias for set_static_transform
        template <typename To, typename From, typename T = double>
        inline void set_static(const std::string &to_frame, const std::string &from_frame,
                               const concord::frame::Transform<To, From, T> &tf) {
            set_static_transform<To, From, T>(to_frame, from_frame, tf);
        }

        // ========================================================================
        // Transform Lookup
        // ========================================================================

        /**
         * @brief Lookup transform at a specific time with interpolation and path composition (double)
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param timestamp Time in seconds
         * @return Composed transform, or nullopt if unavailable
         */
        inline std::optional<GenericTransform> lookup(const std::string &to_frame, const std::string &from_frame,
                                                      double timestamp) const {
            // Same frame -> identity
            if (to_frame == from_frame) {
                if (!m_graph.has_frame(to_frame)) {
                    return std::nullopt;
                }
                return std::optional<GenericTransform>(GenericTransform::identity());
            }

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

            // Compose transforms along path at given time
            return compose_path_at_time(path, timestamp);
        }

        /**
         * @brief Lookup transform at a specific time with interpolation and path composition (int64)
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param timestamp Time in nanoseconds
         * @return Composed transform, or nullopt if unavailable
         */
        inline std::optional<GenericTransform> lookup(const std::string &to_frame, const std::string &from_frame,
                                                      int64_t timestamp) const {
            return lookup(to_frame, from_frame, static_cast<double>(timestamp) / 1e9);
        }

        /**
         * @brief Lookup the latest available transform
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @return Latest composed transform, or nullopt if unavailable
         */
        inline std::optional<GenericTransform> lookup_latest(const std::string &to_frame,
                                                             const std::string &from_frame) const {
            // Same frame -> identity
            if (to_frame == from_frame) {
                if (!m_graph.has_frame(to_frame)) {
                    return std::nullopt;
                }
                return std::optional<GenericTransform>(GenericTransform::identity());
            }

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

            // Find the minimum "latest" time across all edges, then compose at that time
            auto common_time = find_common_latest_time(path);
            if (!common_time) {
                return std::nullopt;
            }

            return compose_path_at_time(path, *common_time);
        }

        /**
         * @brief Check if a direct transform exists between frames
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @return true if transform exists (static or dynamic)
         */
        inline bool has_transform(const std::string &to_frame, const std::string &from_frame) const {
            auto from_opt = m_graph.get_frame_id(from_frame);
            auto to_opt = m_graph.get_frame_id(to_frame);

            if (!from_opt || !to_opt) {
                return false;
            }

            EdgeKey edge_key = make_edge_key(*from_opt, *to_opt);
            return m_static_edges.count(edge_key) > 0 || m_dynamic_transforms.count(edge_key) > 0;
        }

        /**
         * @brief Check if transform is available at a specific time (double)
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param timestamp Time in seconds
         * @return true if transform can be computed at this time
         */
        inline bool can_transform(const std::string &to_frame, const std::string &from_frame, double timestamp) const {
            if (to_frame == from_frame) {
                return m_graph.has_frame(to_frame);
            }

            auto from_opt = m_graph.get_frame_id(from_frame);
            auto to_opt = m_graph.get_frame_id(to_frame);

            if (!from_opt || !to_opt) {
                return false;
            }

            auto path = find_path(*from_opt, *to_opt);
            if (path.empty()) {
                return false;
            }

            // Check each edge in path
            for (size_t i = 0; i < path.size() - 1; ++i) {
                EdgeKey edge_key = make_edge_key(path[i], path[i + 1]);

                // Static edges are always available
                if (m_static_edges.count(edge_key) > 0) {
                    continue;
                }

                // Dynamic edges need to have data at this time
                auto it = m_dynamic_transforms.find(edge_key);
                if (it == m_dynamic_transforms.end() || !it->second.in_range(timestamp)) {
                    return false;
                }
            }

            return true;
        }

        /**
         * @brief Check if transform is available at a specific time (int64)
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param timestamp Time in nanoseconds
         * @return true if transform can be computed at this time
         */
        inline bool can_transform(const std::string &to_frame, const std::string &from_frame, int64_t timestamp) const {
            return can_transform(to_frame, from_frame, static_cast<double>(timestamp) / 1e9);
        }

        /**
         * @brief Get valid time range for a transform path
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @return Optional pair of (min_start, max_end) timestamps in seconds, or nullopt if unavailable
         */
        inline std::optional<std::pair<double, double>> time_range(const std::string &to_frame,
                                                                   const std::string &from_frame) const {
            if (to_frame == from_frame) {
                if (!m_graph.has_frame(to_frame)) {
                    return std::nullopt;
                }
                // Identity transform is always available
                return std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
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

            // Find intersection of all edge time ranges
            double range_start = -std::numeric_limits<double>::max();
            double range_end = std::numeric_limits<double>::max();

            for (size_t i = 0; i < path.size() - 1; ++i) {
                EdgeKey edge_key = make_edge_key(path[i], path[i + 1]);

                // Static edges don't constrain the range
                if (m_static_edges.count(edge_key) > 0) {
                    continue;
                }

                auto it = m_dynamic_transforms.find(edge_key);
                if (it == m_dynamic_transforms.end() || it->second.empty()) {
                    return std::nullopt; // No data for this edge
                }

                auto edge_range = it->second.time_range();
                if (!edge_range) {
                    return std::nullopt;
                }

                range_start = std::max(range_start, edge_range->first);
                range_end = std::min(range_end, edge_range->second);
            }

            if (range_start > range_end) {
                return std::nullopt; // No overlapping range
            }

            return std::make_pair(range_start, range_end);
        }

        // ========================================================================
        // Delegated Methods from FrameGraph
        // ========================================================================

        inline bool has_frame(const std::string &name) const { return m_graph.has_frame(name); }

        template <typename Frame> inline bool has_frame() const { return m_graph.has_frame<Frame>(); }

        inline std::vector<std::string> frame_names() const { return m_graph.frame_names(); }

        inline size_t frame_count() const { return m_graph.frame_count(); }

        inline void clear() {
            m_graph.clear();
            m_dynamic_transforms.clear();
            m_static_transforms.clear();
            m_static_edges.clear();
            m_edge_directions.clear();
            m_path_cache.clear();
        }

        // Access underlying graph (for advanced use)
        inline const FrameGraph &graph() const { return m_graph; }
        inline FrameGraph &graph() { return m_graph; }

      private:
        FrameGraph m_graph;

        // Edge key for transform storage (order-independent)
        using EdgeKey = std::pair<VertexId, VertexId>;

        struct EdgeKeyHash {
            inline size_t operator()(const EdgeKey &k) const {
                return std::hash<VertexId>{}(k.first) ^ (std::hash<VertexId>{}(k.second) << 1);
            }
        };

        static inline EdgeKey make_edge_key(VertexId a, VertexId b) { return (a < b) ? EdgeKey{a, b} : EdgeKey{b, a}; }

        // Transform buffers per edge (dynamic transforms)
        std::unordered_map<EdgeKey, TimedTransformBuffer<100>, EdgeKeyHash> m_dynamic_transforms;

        // Static transforms (no time history)
        std::unordered_map<EdgeKey, GenericTransform, EdgeKeyHash> m_static_transforms;

        // Track which edges are static
        std::unordered_set<EdgeKey, EdgeKeyHash> m_static_edges;

        // Store original direction of transforms (from_id, to_id)
        std::unordered_map<EdgeKey, std::pair<VertexId, VertexId>, EdgeKeyHash> m_edge_directions;

        // Path cache
        using PathCacheKey = std::pair<VertexId, VertexId>;
        mutable std::unordered_map<PathCacheKey, std::vector<VertexId>, EdgeKeyHash> m_path_cache;

        // ========================================================================
        // Path Finding using BFS
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
        // Transform Composition Helpers
        // ========================================================================

        /**
         * @brief Get transform for a single edge at a specific time
         * @param from Source vertex
         * @param to Target vertex
         * @param timestamp Time in seconds
         * @return Transform, or nullopt if unavailable
         */
        inline std::optional<GenericTransform> get_edge_transform_at_time(VertexId from, VertexId to,
                                                                          double timestamp) const {
            EdgeKey edge_key = make_edge_key(from, to);

            // Check if static
            if (m_static_edges.count(edge_key) > 0) {
                auto it = m_static_transforms.find(edge_key);
                if (it != m_static_transforms.end()) {
                    return std::optional<GenericTransform>(get_directed_transform(edge_key, it->second, from, to));
                }
                return std::nullopt;
            }

            // Check dynamic
            auto it = m_dynamic_transforms.find(edge_key);
            if (it != m_dynamic_transforms.end()) {
                auto tf_opt = it->second.lookup(timestamp);
                if (tf_opt) {
                    return std::optional<GenericTransform>(get_directed_transform(edge_key, *tf_opt, from, to));
                }
            }

            return std::nullopt;
        }

        /**
         * @brief Get transform in correct direction (may need to invert)
         */
        inline GenericTransform get_directed_transform(const EdgeKey &edge_key, const GenericTransform &stored_tf,
                                                       VertexId from, VertexId to) const {
            auto dir_it = m_edge_directions.find(edge_key);
            if (dir_it != m_edge_directions.end()) {
                // Transform was stored as (stored_from -> stored_to)
                // We want (from -> to)
                if (dir_it->second.first == from && dir_it->second.second == to) {
                    // Same direction
                    return stored_tf;
                } else {
                    // Inverse direction
                    return stored_tf.inverse();
                }
            }
            // Fallback: assume stored direction matches
            return stored_tf;
        }

        /**
         * @brief Find the minimum "latest" time across all edges in a path
         */
        inline std::optional<double> find_common_latest_time(const std::vector<VertexId> &path) const {
            if (path.size() <= 1) {
                return 0.0; // Identity transform
            }

            double min_latest = std::numeric_limits<double>::max();
            bool has_dynamic = false;

            for (size_t i = 0; i < path.size() - 1; ++i) {
                EdgeKey edge_key = make_edge_key(path[i], path[i + 1]);

                // Static edges don't affect the time
                if (m_static_edges.count(edge_key) > 0) {
                    continue;
                }

                // Dynamic edge
                auto it = m_dynamic_transforms.find(edge_key);
                if (it == m_dynamic_transforms.end() || it->second.empty()) {
                    return std::nullopt;
                }

                has_dynamic = true;
                auto range = it->second.time_range();
                if (range) {
                    min_latest = std::min(min_latest, range->second);
                }
            }

            if (!has_dynamic) {
                return 0.0; // All static
            }

            return min_latest;
        }

        /**
         * @brief Compose transforms along a path at a given time
         */
        inline std::optional<GenericTransform> compose_path_at_time(const std::vector<VertexId> &path,
                                                                    double timestamp) const {
            if (path.empty()) {
                return std::nullopt;
            }

            if (path.size() == 1) {
                return std::optional<GenericTransform>(GenericTransform::identity());
            }

            GenericTransform result = GenericTransform::identity();

            for (size_t i = 0; i < path.size() - 1; ++i) {
                VertexId current = path[i];
                VertexId next = path[i + 1];

                auto edge_tf_opt = get_edge_transform_at_time(current, next, timestamp);
                if (!edge_tf_opt) {
                    return std::nullopt;
                }

                // Compose: result = edge_tf * result
                result = (*edge_tf_opt) * result;
            }

            return std::optional<GenericTransform>(result);
        }
    };

} // namespace concord::transform
