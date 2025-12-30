#pragma once

#include "../frame/transform.hpp"
#include <any>
#include <graphix/vertex/algorithms/bfs.hpp>
#include <graphix/vertex/graph.hpp>
#include <optional>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace concord::transform {

    // ============================================================================
    // GenericTransform - Runtime transform representation
    //
    // This stores rotation (as quaternion) and translation in a type-erased way
    // that can be composed at runtime without knowing the original frame types.
    // ============================================================================
    struct GenericTransform {
        dp::Quaternion rotation{1.0, 0.0, 0.0, 0.0}; // Identity quaternion (w,x,y,z)
        dp::Point translation{0.0, 0.0, 0.0};

        GenericTransform() = default;
        GenericTransform(const dp::Quaternion &q, const dp::Point &t) : rotation(q.normalized()), translation(t) {}

        // Identity transform
        static inline GenericTransform identity() { return GenericTransform{}; }

        // Inverse transform
        inline GenericTransform inverse() const {
            dp::Quaternion q_inv = rotation.conjugate();
            // Rotate translation by inverse rotation, then negate
            dp::Quaternion t_quat{0.0, translation.x, translation.y, translation.z};
            dp::Quaternion rotated = q_inv * t_quat * rotation;
            return GenericTransform{q_inv, dp::Point{-rotated.x, -rotated.y, -rotated.z}};
        }

        // Apply transform to a point: p' = R * p + t
        inline dp::Point apply(const dp::Point &p) const {
            dp::Quaternion p_quat{0.0, p.x, p.y, p.z};
            dp::Quaternion rotated = rotation * p_quat * rotation.conjugate();
            return dp::Point{rotated.x + translation.x, rotated.y + translation.y, rotated.z + translation.z};
        }

        // Compose transforms: this * other
        // Result transforms from other's source to this's target
        inline GenericTransform operator*(const GenericTransform &other) const {
            // Combined rotation: R_ab * R_bc = R_ac
            dp::Quaternion q_combined = rotation * other.rotation;

            // Combined translation: t_ac = R_ab * t_bc + t_ab
            dp::Quaternion t_quat{0.0, other.translation.x, other.translation.y, other.translation.z};
            dp::Quaternion rotated = rotation * t_quat * rotation.conjugate();
            dp::Point t_combined{rotated.x + translation.x, rotated.y + translation.y, rotated.z + translation.z};

            return GenericTransform{q_combined, t_combined};
        }
    };

    // ============================================================================
    // FrameInfo - Vertex property storing frame metadata
    // ============================================================================
    struct FrameInfo {
        std::string name;        // Human-readable frame name
        std::type_index type_id; // Type info for the frame tag

        FrameInfo() : name(""), type_id(typeid(void)) {}
        FrameInfo(const std::string &n, std::type_index t) : name(n), type_id(t) {}
    };

    // ============================================================================
    // FrameGraph - Graph storing frames and transforms between them
    //
    // Design philosophy:
    // - Frame tags are compile-time empty structs for type safety
    // - Uses graphix::vertex::Graph at runtime for frame storage and path finding
    // - Transform data stored on edges
    // - Supports both type-safe (template) and name-based (string) APIs
    // ============================================================================
    class FrameGraph {
      public:
        using GraphType = graphix::vertex::Graph<FrameInfo>;
        using VertexId = graphix::Key;

      private:
        using EdgeKey = std::pair<VertexId, VertexId>;

        // Internal transform data with vertex direction info
        struct InternalTransformData {
            std::any transform;       // Type-erased original transform
            GenericTransform generic; // Runtime-usable transform
            VertexId from_id = 0;
            VertexId to_id = 0;
        };

        // Hash for EdgeKey
        struct EdgeKeyHash {
            size_t operator()(const EdgeKey &key) const {
                return std::hash<VertexId>{}(key.first) ^ (std::hash<VertexId>{}(key.second) << 1);
            }
        };

        GraphType m_graph;
        std::unordered_map<std::string, VertexId> m_name_to_vertex;
        std::unordered_map<VertexId, std::string> m_vertex_to_name;
        std::unordered_map<std::string, std::type_index> m_name_to_type;
        std::unordered_map<std::type_index, VertexId> m_type_to_vertex;
        std::unordered_map<EdgeKey, InternalTransformData, EdgeKeyHash> m_transforms;

        // Create canonical edge key (smaller vertex first for consistency)
        static inline EdgeKey make_edge_key(VertexId a, VertexId b) {
            return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
        }

      public:
        FrameGraph() = default;

        // ========================================================================
        // Frame Registration - Name-based API (for runtime flexibility)
        // ========================================================================

        // Register a frame by name with associated type
        template <typename Frame> inline VertexId register_frame(const std::string &name) {
            // Check if already registered by name
            auto name_it = m_name_to_vertex.find(name);
            if (name_it != m_name_to_vertex.end()) {
                return name_it->second;
            }

            std::type_index type_id = std::type_index(typeid(Frame));

            // Create frame info and add vertex
            FrameInfo info(name, type_id);
            VertexId vertex_id = m_graph.add_vertex(info);

            // Store all mappings
            m_name_to_vertex[name] = vertex_id;
            m_vertex_to_name[vertex_id] = name;
            m_name_to_type.insert_or_assign(name, type_id);
            m_type_to_vertex.insert_or_assign(type_id, vertex_id);

            return vertex_id;
        }

        // Check if a frame is registered by name
        inline bool has_frame(const std::string &name) const {
            return m_name_to_vertex.find(name) != m_name_to_vertex.end();
        }

        // Get vertex ID for a frame name
        inline std::optional<VertexId> get_frame_id(const std::string &name) const {
            auto it = m_name_to_vertex.find(name);
            if (it != m_name_to_vertex.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        // ========================================================================
        // Frame Registration - Type-based API (for compile-time type safety)
        // ========================================================================

        // Check if a frame type is registered
        template <typename Frame> inline bool has_frame() const {
            std::type_index type_id = std::type_index(typeid(Frame));
            return m_type_to_vertex.find(type_id) != m_type_to_vertex.end();
        }

        // Get vertex ID for a frame type
        template <typename Frame> inline std::optional<VertexId> get_frame_id() const {
            std::type_index type_id = std::type_index(typeid(Frame));
            auto it = m_type_to_vertex.find(type_id);
            if (it != m_type_to_vertex.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        // ========================================================================
        // Transform Management - Name-based API
        // ========================================================================

        // Set transform between frames (by name)
        template <typename To, typename From, typename T = double>
        inline void set_transform(const std::string &to_frame, const std::string &from_frame,
                                  const concord::frame::Transform<To, From, T> &tf) {
            // Ensure frames exist
            if (!has_frame(to_frame) || !has_frame(from_frame)) {
                return;
            }

            VertexId from_id = m_name_to_vertex.at(from_frame);
            VertexId to_id = m_name_to_vertex.at(to_frame);

            // Create edge key (order-independent)
            EdgeKey edge_key = make_edge_key(from_id, to_id);

            // Store transform with direction info and generic representation
            InternalTransformData data;
            data.transform = tf;
            data.generic = GenericTransform{tf.rotation.quaternion(), tf.translation};
            data.from_id = from_id;
            data.to_id = to_id;
            m_transforms[edge_key] = data;

            // Add edge to graph if not present
            if (!m_graph.has_edge(from_id, to_id)) {
                m_graph.add_edge(from_id, to_id, 1.0, graphix::vertex::EdgeType::Undirected);
            }
        }

        // Check if a transform exists between two frames (by name)
        inline bool has_transform(const std::string &to_frame, const std::string &from_frame) const {
            auto from_opt = get_frame_id(from_frame);
            auto to_opt = get_frame_id(to_frame);
            if (!from_opt || !to_opt) {
                return false;
            }
            EdgeKey edge_key = make_edge_key(*from_opt, *to_opt);
            return m_transforms.find(edge_key) != m_transforms.end();
        }

        // Get a transform between two frames (by name) - returns GenericTransform
        inline std::optional<GenericTransform> get_transform(const std::string &to_frame,
                                                             const std::string &from_frame) const {
            auto from_opt = get_frame_id(from_frame);
            auto to_opt = get_frame_id(to_frame);
            if (!from_opt || !to_opt) {
                return std::nullopt;
            }

            EdgeKey edge_key = make_edge_key(*from_opt, *to_opt);
            auto it = m_transforms.find(edge_key);
            if (it == m_transforms.end()) {
                return std::nullopt;
            }

            const InternalTransformData &data = it->second;

            // Check direction and return appropriate transform
            if (data.from_id == *from_opt && data.to_id == *to_opt) {
                // Direct match
                return data.generic;
            } else {
                // Inverse direction
                return data.generic.inverse();
            }
        }

        // ========================================================================
        // Transform Management - Type-based API
        // ========================================================================

        // Set transform between frames (requires frames to be already registered)
        template <typename To, typename From, typename T = double>
        inline void set_transform(const concord::frame::Transform<To, From, T> &tf, bool is_static = false) {
            // Get frame IDs
            std::type_index from_type = std::type_index(typeid(From));
            std::type_index to_type = std::type_index(typeid(To));

            // Check if frames are registered by type
            auto from_it = m_type_to_vertex.find(from_type);
            auto to_it = m_type_to_vertex.find(to_type);

            if (from_it == m_type_to_vertex.end() || to_it == m_type_to_vertex.end()) {
                return;
            }

            VertexId from_id = from_it->second;
            VertexId to_id = to_it->second;

            // Create edge key
            EdgeKey edge_key = make_edge_key(from_id, to_id);

            // Store transform with direction info and generic representation
            InternalTransformData data;
            data.transform = tf;
            data.generic = GenericTransform{tf.rotation.quaternion(), tf.translation};
            data.from_id = from_id;
            data.to_id = to_id;
            m_transforms[edge_key] = data;

            // Add edge to graph if not present
            if (!m_graph.has_edge(from_id, to_id)) {
                m_graph.add_edge(from_id, to_id, 1.0, graphix::vertex::EdgeType::Undirected);
            }
        }

        // Get transform between frame types (direct edge only)
        template <typename To, typename From, typename T = double>
        inline std::optional<concord::frame::Transform<To, From, T>> get_transform() const {
            auto from_opt = get_frame_id<From>();
            auto to_opt = get_frame_id<To>();

            if (!from_opt || !to_opt) {
                return std::nullopt;
            }

            EdgeKey edge_key = make_edge_key(*from_opt, *to_opt);
            auto it = m_transforms.find(edge_key);

            if (it == m_transforms.end()) {
                return std::nullopt;
            }

            const InternalTransformData &data = it->second;

            // Check direction and return appropriate transform
            if (data.from_id == *from_opt && data.to_id == *to_opt) {
                // Direct match - return as-is
                try {
                    return std::any_cast<concord::frame::Transform<To, From, T>>(data.transform);
                } catch (const std::bad_any_cast &) {
                    return std::nullopt;
                }
            } else {
                // Inverse direction - need to invert the transform
                try {
                    auto stored = std::any_cast<concord::frame::Transform<From, To, T>>(data.transform);
                    return stored.inverse();
                } catch (const std::bad_any_cast &) {
                    return std::nullopt;
                }
            }
        }

        // Check if direct transform exists between two frame types
        template <typename To, typename From> inline bool has_transform() const {
            auto from_opt = get_frame_id<From>();
            auto to_opt = get_frame_id<To>();

            if (!from_opt || !to_opt) {
                return false;
            }

            EdgeKey edge_key = make_edge_key(*from_opt, *to_opt);
            return m_transforms.find(edge_key) != m_transforms.end();
        }

        // ========================================================================
        // Raw Transform Access (for TransformTree)
        // ========================================================================

        // Get generic transform between two vertices
        inline std::optional<GenericTransform> get_generic_transform(VertexId from, VertexId to) const {
            EdgeKey edge_key = make_edge_key(from, to);
            auto it = m_transforms.find(edge_key);
            if (it == m_transforms.end()) {
                return std::nullopt;
            }

            const InternalTransformData &data = it->second;
            if (data.from_id == from && data.to_id == to) {
                return data.generic;
            } else {
                return data.generic.inverse();
            }
        }

        // Get transform direction info
        inline bool get_transform_direction(VertexId from, VertexId to, VertexId &stored_from,
                                            VertexId &stored_to) const {
            EdgeKey edge_key = make_edge_key(from, to);
            auto it = m_transforms.find(edge_key);
            if (it != m_transforms.end()) {
                stored_from = it->second.from_id;
                stored_to = it->second.to_id;
                return true;
            }
            return false;
        }

        // ========================================================================
        // Graph Queries
        // ========================================================================

        // Get all registered frame names
        inline std::vector<std::string> frame_names() const {
            std::vector<std::string> names;
            names.reserve(m_vertex_to_name.size());
            for (const auto &[id, name] : m_vertex_to_name) {
                names.push_back(name);
            }
            return names;
        }

        // Get number of frames
        inline size_t frame_count() const { return m_graph.vertex_count(); }

        // Get number of transforms (edges)
        inline size_t transform_count() const { return m_transforms.size(); }

        // Clear all frames and transforms
        inline void clear() {
            m_graph.clear();
            m_name_to_vertex.clear();
            m_vertex_to_name.clear();
            m_name_to_type.clear();
            m_type_to_vertex.clear();
            m_transforms.clear();
        }

        // ========================================================================
        // Advanced Access
        // ========================================================================

        // Get frame info by vertex ID
        inline const FrameInfo &get_frame_info(VertexId vertex_id) const { return m_graph[vertex_id]; }

        // Get frame name by vertex ID
        inline std::string get_frame_name(VertexId vertex_id) const {
            auto it = m_vertex_to_name.find(vertex_id);
            if (it != m_vertex_to_name.end()) {
                return it->second;
            }
            return "";
        }

        // Check if edge exists between vertices
        inline bool has_edge(VertexId from, VertexId to) const { return m_graph.has_edge(from, to); }

        // Get neighbors of a vertex
        inline std::vector<VertexId> neighbors(VertexId v) const { return m_graph.neighbors(v); }

        // Access underlying graphix graph (for algorithms like BFS)
        inline const GraphType &graph() const { return m_graph; }
        inline GraphType &graph() { return m_graph; }

        // ========================================================================
        // Path Finding
        // ========================================================================

        // Find path between two frame types using BFS
        template <typename To, typename From> inline std::vector<VertexId> find_path() const {
            auto from_opt = get_frame_id<From>();
            auto to_opt = get_frame_id<To>();

            if (!from_opt || !to_opt) {
                return {};
            }

            auto bfs_result = graphix::vertex::algorithms::bfs(m_graph, *from_opt, *to_opt);

            if (!bfs_result.target_found && bfs_result.distance.find(*to_opt) == bfs_result.distance.end()) {
                return {};
            }

            return graphix::vertex::algorithms::reconstruct_path(bfs_result, *from_opt, *to_opt);
        }

        // Check if path exists between two frame types
        template <typename To, typename From> inline bool has_path() const {
            auto path = find_path<To, From>();
            return !path.empty();
        }
    };

} // namespace concord::transform
