#pragma once

#include "temporal.hpp"
#include "tree.hpp"
#include <algorithm>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace concord::transform {

    // ============================================================================
    // Callback types for transform updates
    // ============================================================================

    // Callback type for specific edge updates
    using TransformCallback = std::function<void(const GenericTransform &)>;

    // Callback type for any transform update (receives frame names and transform)
    using AnyTransformCallback =
        std::function<void(const std::string &, const std::string &, const GenericTransform &)>;

    // ============================================================================
    // TransformListenerMixin - Adds listener support to transform trees
    //
    // This mixin class provides:
    // - Registration of callbacks for specific edge updates
    // - Registration of callbacks for any transform update
    // - Removal of callbacks by ID or by edge
    // - Thread-safe callback management
    // ============================================================================
    template <typename Derived> class TransformListenerMixin {
      public:
        TransformListenerMixin() = default;

        // ========================================================================
        // Listener Registration
        // ========================================================================

        /**
         * @brief Register callback for specific edge updates (by frame names)
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param callback Function to call when transform is updated
         * @return Unique listener ID for later removal
         */
        inline size_t on_update(const std::string &to_frame, const std::string &from_frame,
                                TransformCallback callback) {
            std::lock_guard<std::mutex> lock(m_listener_mutex);

            size_t id = m_next_listener_id++;
            StringEdgeKey key = make_string_edge_key(to_frame, from_frame);

            m_edge_listeners[key].emplace_back(id, std::move(callback));
            m_listener_to_edge[id] = key;

            return id;
        }

        /**
         * @brief Register callback for any transform update
         * @param callback Function to call when any transform is updated
         * @return Unique listener ID for later removal
         */
        inline size_t on_any_update(AnyTransformCallback callback) {
            std::lock_guard<std::mutex> lock(m_listener_mutex);

            size_t id = m_next_listener_id++;
            m_any_listeners.emplace_back(id, std::move(callback));

            return id;
        }

        // ========================================================================
        // Listener Removal
        // ========================================================================

        /**
         * @brief Remove a specific callback by ID
         * @param listener_id ID returned from on_update or on_any_update
         */
        inline void remove_listener(size_t listener_id) {
            std::lock_guard<std::mutex> lock(m_listener_mutex);

            // Check edge listeners
            auto edge_it = m_listener_to_edge.find(listener_id);
            if (edge_it != m_listener_to_edge.end()) {
                auto &listeners = m_edge_listeners[edge_it->second];
                listeners.erase(std::remove_if(listeners.begin(), listeners.end(),
                                               [listener_id](const auto &pair) { return pair.first == listener_id; }),
                                listeners.end());
                m_listener_to_edge.erase(edge_it);
                return;
            }

            // Check any listeners
            m_any_listeners.erase(std::remove_if(m_any_listeners.begin(), m_any_listeners.end(),
                                                 [listener_id](const auto &pair) { return pair.first == listener_id; }),
                                  m_any_listeners.end());
        }

        /**
         * @brief Remove all listeners for a specific edge
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         */
        inline void remove_listeners(const std::string &to_frame, const std::string &from_frame) {
            std::lock_guard<std::mutex> lock(m_listener_mutex);

            StringEdgeKey key = make_string_edge_key(to_frame, from_frame);

            // Remove from listener_to_edge map
            auto it = m_edge_listeners.find(key);
            if (it != m_edge_listeners.end()) {
                for (const auto &[id, _] : it->second) {
                    m_listener_to_edge.erase(id);
                }
                m_edge_listeners.erase(it);
            }
        }

        /**
         * @brief Remove all listeners
         */
        inline void clear_listeners() {
            std::lock_guard<std::mutex> lock(m_listener_mutex);

            m_edge_listeners.clear();
            m_any_listeners.clear();
            m_listener_to_edge.clear();
        }

        /**
         * @brief Get number of registered edge listeners
         * @return Total count of edge-specific listeners
         */
        inline size_t edge_listener_count() const {
            std::lock_guard<std::mutex> lock(m_listener_mutex);
            size_t count = 0;
            for (const auto &[_, listeners] : m_edge_listeners) {
                count += listeners.size();
            }
            return count;
        }

        /**
         * @brief Get number of registered any-update listeners
         * @return Count of any-update listeners
         */
        inline size_t any_listener_count() const {
            std::lock_guard<std::mutex> lock(m_listener_mutex);
            return m_any_listeners.size();
        }

        // ========================================================================
        // Callback Notification (called by derived classes)
        // ========================================================================

        /**
         * @brief Notify all relevant listeners of a transform update
         * @param to_frame Target frame name
         * @param from_frame Source frame name
         * @param tf The updated transform
         */
        inline void notify_listeners(const std::string &to_frame, const std::string &from_frame,
                                     const GenericTransform &tf) {
            std::lock_guard<std::mutex> lock(m_listener_mutex);

            // Notify edge-specific listeners
            StringEdgeKey key = make_string_edge_key(to_frame, from_frame);
            auto it = m_edge_listeners.find(key);
            if (it != m_edge_listeners.end()) {
                for (const auto &[_, callback] : it->second) {
                    callback(tf);
                }
            }

            // Notify any-update listeners
            for (const auto &[_, callback] : m_any_listeners) {
                callback(to_frame, from_frame, tf);
            }
        }

      private:
        // String-based edge key (order-independent for consistent lookup)
        using StringEdgeKey = std::pair<std::string, std::string>;

        struct StringEdgeKeyHash {
            inline size_t operator()(const StringEdgeKey &k) const {
                return std::hash<std::string>{}(k.first) ^ (std::hash<std::string>{}(k.second) << 1);
            }
        };

        static inline StringEdgeKey make_string_edge_key(const std::string &a, const std::string &b) {
            return (a < b) ? StringEdgeKey{a, b} : StringEdgeKey{b, a};
        }

        // Listener storage
        std::unordered_map<StringEdgeKey, std::vector<std::pair<size_t, TransformCallback>>, StringEdgeKeyHash>
            m_edge_listeners;
        std::vector<std::pair<size_t, AnyTransformCallback>> m_any_listeners;
        std::unordered_map<size_t, StringEdgeKey> m_listener_to_edge;

        // Atomic counter for unique listener IDs
        size_t m_next_listener_id = 1;

        // Mutex for thread safety
        mutable std::mutex m_listener_mutex;
    };

    // ============================================================================
    // ListenableTransformTree - TransformTree with listener support
    // ============================================================================

    /**
     * @brief Transform tree with path finding and listener support
     *
     * Extends TransformTree functionality with:
     * - Callbacks for specific edge updates
     * - Callbacks for any transform update
     * - Thread-safe listener management
     */
    class ListenableTransformTree : public TransformListenerMixin<ListenableTransformTree> {
      public:
        using MixinBase = TransformListenerMixin<ListenableTransformTree>;
        using VertexId = FrameGraph::VertexId;

        ListenableTransformTree() = default;

        // ========================================================================
        // Frame registration (delegated to internal tree)
        // ========================================================================

        template <typename Frame> inline VertexId register_frame(const std::string &name) {
            return m_tree.register_frame<Frame>(name);
        }

        // ========================================================================
        // Transform management with listener notification
        // ========================================================================

        /**
         * @brief Set transform between frames and notify listeners
         */
        template <typename To, typename From, typename T = double>
        inline void set_transform(const std::string &to_frame, const std::string &from_frame,
                                  const concord::frame::Transform<To, From, T> &tf) {
            m_tree.set_transform(to_frame, from_frame, tf);

            // Notify listeners
            GenericTransform generic(tf.rotation.quaternion(), tf.translation);
            notify_listeners(to_frame, from_frame, generic);
        }

        // ========================================================================
        // Lookup operations (delegated to internal tree)
        // ========================================================================

        inline std::optional<GenericTransform> lookup(const std::string &to_frame,
                                                      const std::string &from_frame) const {
            return m_tree.lookup(to_frame, from_frame);
        }

        inline bool can_transform(const std::string &to_frame, const std::string &from_frame) const {
            return m_tree.can_transform(to_frame, from_frame);
        }

        inline std::optional<std::vector<std::string>> get_path(const std::string &from_frame,
                                                                const std::string &to_frame) const {
            return m_tree.get_path(from_frame, to_frame);
        }

        template <typename To, typename From, typename T = double>
        inline std::optional<concord::frame::Transform<To, From, T>> lookup() const {
            return m_tree.lookup<To, From, T>();
        }

        template <typename To, typename From> inline bool can_transform() const {
            return m_tree.can_transform<To, From>();
        }

        // ========================================================================
        // Delegated methods
        // ========================================================================

        inline bool has_frame(const std::string &name) const { return m_tree.has_frame(name); }

        template <typename Frame> inline bool has_frame() const { return m_tree.has_frame<Frame>(); }

        inline std::vector<std::string> frame_names() const { return m_tree.frame_names(); }

        inline size_t frame_count() const { return m_tree.frame_count(); }

        inline size_t transform_count() const { return m_tree.transform_count(); }

        inline void clear() {
            m_tree.clear();
            clear_listeners();
        }

        inline const FrameGraph &graph() const { return m_tree.graph(); }
        inline FrameGraph &graph() { return m_tree.graph(); }

      private:
        TransformTree m_tree;
    };

    // ============================================================================
    // ListenableTimedTransformTree - TimedTransformTree with listener support
    // ============================================================================

    /**
     * @brief Timed transform tree with listener support
     *
     * Extends TimedTransformTree functionality with:
     * - Callbacks for specific edge updates
     * - Callbacks for any transform update
     * - Thread-safe listener management
     */
    class ListenableTimedTransformTree : public TransformListenerMixin<ListenableTimedTransformTree> {
      public:
        using VertexId = FrameGraph::VertexId;

        ListenableTimedTransformTree() = default;

        // ========================================================================
        // Frame registration
        // ========================================================================

        template <typename Frame> inline VertexId register_frame(const std::string &name) {
            return m_tree.register_frame<Frame>(name);
        }

        // ========================================================================
        // Dynamic transform management with listener notification
        // ========================================================================

        template <typename To, typename From, typename T = double>
        inline void set_transform(const std::string &to_frame, const std::string &from_frame,
                                  const concord::frame::Transform<To, From, T> &tf, double timestamp) {
            m_tree.set_transform(to_frame, from_frame, tf, timestamp);

            // Notify listeners
            GenericTransform generic(tf.rotation.quaternion(), tf.translation);
            notify_listeners(to_frame, from_frame, generic);
        }

        template <typename To, typename From, typename T = double>
        inline void set(int64_t timestamp, const std::string &to_frame, const std::string &from_frame,
                        const concord::frame::Transform<To, From, T> &tf) {
            set_transform<To, From, T>(to_frame, from_frame, tf, static_cast<double>(timestamp) / 1e9);
        }

        template <typename To, typename From, typename T = double>
        inline void set(const std::string &to_frame, const std::string &from_frame,
                        const dp::Stamp<concord::frame::Transform<To, From, T>> &stamped) {
            set_transform<To, From, T>(to_frame, from_frame, stamped.value,
                                       static_cast<double>(stamped.timestamp) / 1e9);
        }

        // ========================================================================
        // Static transform management with listener notification
        // ========================================================================

        template <typename To, typename From, typename T = double>
        inline void set_static_transform(const std::string &to_frame, const std::string &from_frame,
                                         const concord::frame::Transform<To, From, T> &tf) {
            m_tree.set_static_transform(to_frame, from_frame, tf);

            // Notify listeners
            GenericTransform generic(tf.rotation.quaternion(), tf.translation);
            notify_listeners(to_frame, from_frame, generic);
        }

        template <typename To, typename From, typename T = double>
        inline void set_static(const std::string &to_frame, const std::string &from_frame,
                               const concord::frame::Transform<To, From, T> &tf) {
            set_static_transform<To, From, T>(to_frame, from_frame, tf);
        }

        // ========================================================================
        // Lookup operations
        // ========================================================================

        inline std::optional<GenericTransform> lookup(const std::string &to_frame, const std::string &from_frame,
                                                      double timestamp) const {
            return m_tree.lookup(to_frame, from_frame, timestamp);
        }

        inline std::optional<GenericTransform> lookup(const std::string &to_frame, const std::string &from_frame,
                                                      int64_t timestamp) const {
            return m_tree.lookup(to_frame, from_frame, timestamp);
        }

        inline std::optional<GenericTransform> lookup_latest(const std::string &to_frame,
                                                             const std::string &from_frame) const {
            return m_tree.lookup_latest(to_frame, from_frame);
        }

        inline bool has_transform(const std::string &to_frame, const std::string &from_frame) const {
            return m_tree.has_transform(to_frame, from_frame);
        }

        inline bool can_transform(const std::string &to_frame, const std::string &from_frame, double timestamp) const {
            return m_tree.can_transform(to_frame, from_frame, timestamp);
        }

        inline bool can_transform(const std::string &to_frame, const std::string &from_frame, int64_t timestamp) const {
            return m_tree.can_transform(to_frame, from_frame, timestamp);
        }

        inline std::optional<std::pair<double, double>> time_range(const std::string &to_frame,
                                                                   const std::string &from_frame) const {
            return m_tree.time_range(to_frame, from_frame);
        }

        // ========================================================================
        // Delegated methods
        // ========================================================================

        inline bool has_frame(const std::string &name) const { return m_tree.has_frame(name); }

        template <typename Frame> inline bool has_frame() const { return m_tree.has_frame<Frame>(); }

        inline std::vector<std::string> frame_names() const { return m_tree.frame_names(); }

        inline size_t frame_count() const { return m_tree.frame_count(); }

        inline void clear() {
            m_tree.clear();
            clear_listeners();
        }

        inline const FrameGraph &graph() const { return m_tree.graph(); }
        inline FrameGraph &graph() { return m_tree.graph(); }

      private:
        TimedTransformTree m_tree;
    };

} // namespace concord::transform
