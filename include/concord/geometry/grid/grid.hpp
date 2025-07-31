#pragma once

#include "../../core/types.hpp"
#include "../polygon/polygon.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace concord {

    template <typename T> class Grid {
        using size_type = std::size_t;
        using value_type = T;
        using const_reference = const T &;
        using reference = T &;

      private:
        size_type rows_{0};
        size_type cols_{0};
        double inradius_{0.0};
        bool centered_{false};
        bool reverse_y_{false};
        concord::Pose shift_{};
        std::vector<T> data_;

        // Precomputed transform constants
        double width_{0.0};
        double height_{0.0};
        double cos_yaw_{1.0};
        double sin_yaw_{0.0};
        double half_width_{0.0};
        double half_height_{0.0};
        double offset_x_{0.0};   // Pre-transformed x offset
        double offset_y_{0.0};   // Pre-transformed y offset
        bool is_identity_{true}; // Fast path flag

        // Precomputed local coordinates
        std::vector<double> x_coords_; // Local x coordinates (size = cols_)
        std::vector<double> y_coords_; // Local y coordinates (size = rows_)

      public:
        Grid() = default;

        Grid(size_type rows, size_type cols, double radius, bool centered, concord::Pose shift, bool reverse_y = false)
            : rows_{rows}, cols_{cols}, inradius_{radius}, centered_{centered}, shift_{shift}, reverse_y_{reverse_y},
              data_(rows * cols) {
            // Validate constructor parameters
            if (rows == 0 || cols == 0) {
                throw std::invalid_argument("Grid dimensions must be positive");
            }
            if (radius <= 0.0) {
                throw std::invalid_argument("Grid radius must be positive");
            }

            // Precompute transform constants
            height_ = centered_ ? rows_ * inradius_ : 0.0;
            width_ = centered_ ? cols_ * inradius_ : 0.0;
            cos_yaw_ = std::cos(shift_.angle.yaw);
            sin_yaw_ = std::sin(shift_.angle.yaw);

            // Additional precomputed constants
            half_width_ = width_ * 0.5;
            half_height_ = height_ * 0.5;

            // Check for identity transform (no rotation, no translation)
            constexpr double epsilon = 1e-15;
            is_identity_ = (std::abs(cos_yaw_ - 1.0) < epsilon) && (std::abs(sin_yaw_) < epsilon) &&
                           (std::abs(shift_.point.x) < epsilon) && (std::abs(shift_.point.y) < epsilon);

            // Precompute final offsets
            offset_x_ = -half_width_ + shift_.point.x;
            offset_y_ = -half_height_ + shift_.point.y;

            // Precompute local coordinates
            x_coords_.resize(cols_);
            y_coords_.resize(rows_);
            for (size_type c = 0; c < cols_; ++c) {
                x_coords_[c] = (static_cast<double>(c) + 0.5) * inradius_;
            }
            for (size_type r = 0; r < rows_; ++r) {
                if (reverse_y) {
                    // Reverse Y coordinate: row 0 = top (highest Y), row N-1 = bottom (lowest Y)
                    y_coords_[r] = (static_cast<double>(rows_ - 1 - r) + 0.5) * inradius_;
                } else {
                    // Standard Y coordinate: row 0 = bottom (lowest Y), row N-1 = top (highest Y)
                    y_coords_[r] = (static_cast<double>(r) + 0.5) * inradius_;
                }
            }

            // Initialize data with default T values
            std::fill(data_.begin(), data_.end(), T{});
        }

        Grid(const Polygon &poly, double resolution, bool centered = true, bool reverse_y = false)
            : Grid(static_cast<size_type>(std::ceil(poly.get_obb().size.x / resolution)),
                   static_cast<size_type>(std::ceil(poly.get_obb().size.y / resolution)), resolution, centered,
                   poly.get_obb().pose, reverse_y) {}

        // Compute point on-demand (optimized with fast paths)
        [[gnu::hot]] inline Point get_point(size_type r, size_type c) const {
            if (r >= rows_ || c >= cols_) {
                throw std::out_of_range("Grid indices out of bounds");
            }

            // Fast path for identity transform (no rotation, no translation)
            if (is_identity_) [[likely]] {
                return Point{x_coords_[c] + offset_x_, y_coords_[r] + offset_y_, 0.0};
            }

            // General case with rotation/translation using precomputed coordinates
            double local_x = x_coords_[c] - half_width_;
            double local_y = y_coords_[r] - half_height_;

            // Apply rotation and translation
            return Point{cos_yaw_ * local_x - sin_yaw_ * local_y + shift_.point.x,
                         sin_yaw_ * local_x + cos_yaw_ * local_y + shift_.point.y, 0.0};
        }

        // Data access methods
        inline void set_value(size_type r, size_type c, const T &value) { data_[index_checked(r, c)] = value; }

        inline reference operator()(size_type r, size_type c) noexcept { return data_[index(r, c)]; }

        inline const_reference operator()(size_type r, size_type c) const noexcept { return data_[index(r, c)]; }

        inline reference at(size_type r, size_type c) { return data_.at(index_checked(r, c)); }

        inline const_reference at(size_type r, size_type c) const { return data_.at(index_checked(r, c)); }

        inline std::span<T> row(size_type r) {
            if (r >= rows_) {
                throw std::out_of_range("Row index out of bounds");
            }
            return {&data_[index(r, 0)], cols_};
        }

        inline std::span<const T> row(size_type r) const {
            if (r >= rows_) {
                throw std::out_of_range("Row index out of bounds");
            }
            return {&data_[index(r, 0)], cols_};
        }

        constexpr inline size_type index(size_type r, size_type c) const noexcept { return r * cols_ + c; }

        inline size_type index_checked(size_type r, size_type c) const {
            if (r >= rows_ || c >= cols_) {
                throw std::out_of_range("Grid indices out of bounds");
            }
            return r * cols_ + c;
        }

        constexpr size_type rows() const noexcept { return rows_; }
        constexpr size_type cols() const noexcept { return cols_; }
        constexpr double inradius() const noexcept { return inradius_; }

        // Data iterators
        inline auto begin() noexcept { return data_.begin(); }
        inline auto end() noexcept { return data_.end(); }
        inline auto begin() const noexcept { return data_.begin(); }
        inline auto end() const noexcept { return data_.end(); }

        // Optimized flatten_points for bulk point generation
        inline std::vector<std::array<float, 3>> flatten_points() const {
            std::vector<std::array<float, 3>> points;
            const size_type total_points = rows_ * cols_;
            points.resize(total_points); // Pre-allocate to avoid reallocation

            if (is_identity_) [[likely]] {
                // Vectorized identity transform path
                const float inradius_f = static_cast<float>(inradius_);
                const float offset_x_f = static_cast<float>(offset_x_);
                const float offset_y_f = static_cast<float>(offset_y_);

                size_type idx = 0;
                for (size_type r = 0; r < rows_; ++r) {
                    const float base_y = (static_cast<float>(r) + 0.5f) * inradius_f + offset_y_f;
                    for (size_type c = 0; c < cols_; ++c) {
                        const float x = (static_cast<float>(c) + 0.5f) * inradius_f + offset_x_f;
                        points[idx++] = {x, base_y, 0.0f};
                    }
                }
            } else {
                // General case with rotation
                const float inradius_f = static_cast<float>(inradius_);
                const float half_width_f = static_cast<float>(half_width_);
                const float half_height_f = static_cast<float>(half_height_);
                const float cos_yaw_f = static_cast<float>(cos_yaw_);
                const float sin_yaw_f = static_cast<float>(sin_yaw_);
                const float shift_x_f = static_cast<float>(shift_.point.x);
                const float shift_y_f = static_cast<float>(shift_.point.y);

                size_type idx = 0;
                for (size_type r = 0; r < rows_; ++r) {
                    const float local_y_base = (static_cast<float>(r) + 0.5f) * inradius_f - half_height_f;
                    for (size_type c = 0; c < cols_; ++c) {
                        const float local_x = (static_cast<float>(c) + 0.5f) * inradius_f - half_width_f;

                        const float world_x = cos_yaw_f * local_x - sin_yaw_f * local_y_base + shift_x_f;
                        const float world_y = sin_yaw_f * local_x + cos_yaw_f * local_y_base + shift_y_f;

                        points[idx++] = {world_x, world_y, 0.0f};
                    }
                }
            }
            return points;
        }

        inline std::array<concord::Point, 4> corners() const {
            if (rows_ == 0 || cols_ == 0) {
                throw std::runtime_error("Grid is empty; cannot get corners");
            }
            const size_type r0 = 0, r1 = rows_ - 1;
            const size_type c0 = 0, c1 = cols_ - 1;

            return {
                get_point(r0, c0), // top-left
                get_point(r0, c1), // top-right
                get_point(r1, c1), // bottom-right
                get_point(r1, c0)  // bottom-left
            };
        }

        // High-performance polygon overlap using AABB pre-filtering
        inline std::vector<size_type> indices_within(const Polygon &poly) const {
            if (!poly.isConnected()) {
                throw std::invalid_argument("Polygon must have at least 3 vertices");
            }

            // Get polygon's axis-aligned bounding box
            auto aabb = poly.getAABB();

            // Convert AABB world coordinates to grid indices
            auto [r_min_u, r_max_u, c_min_u, c_max_u] = world_to_grid_bounds(aabb.min_point, aabb.max_point);

            // Convert to signed for safe comparison
            const auto r_min_s = static_cast<std::ptrdiff_t>(r_min_u);
            const auto r_max_s = static_cast<std::ptrdiff_t>(r_max_u);
            const auto c_min_s = static_cast<std::ptrdiff_t>(c_min_u);
            const auto c_max_s = static_cast<std::ptrdiff_t>(c_max_u);

            // Early exit if bounding box doesn't overlap grid
            if (r_min_s >= static_cast<std::ptrdiff_t>(rows_) || c_min_s >= static_cast<std::ptrdiff_t>(cols_) ||
                r_max_s < 0 || c_max_s < 0) {
                return {};
            }

            // Clamp bounds to grid limits
            const auto r_min = static_cast<size_type>(std::max(std::ptrdiff_t(0), r_min_s));
            const auto r_max = static_cast<size_type>(std::min(static_cast<std::ptrdiff_t>(rows_ - 1), r_max_s));
            const auto c_min = static_cast<size_type>(std::max(std::ptrdiff_t(0), c_min_s));
            const auto c_max = static_cast<size_type>(std::min(static_cast<std::ptrdiff_t>(cols_ - 1), c_max_s));

            std::vector<size_type> result;
            const size_type bounded_cells = (r_max - r_min + 1) * (c_max - c_min + 1);
            result.reserve(bounded_cells / 2); // Estimate 50% fill rate within bounding box

            // Only test cells within the bounding box
            for (size_type r = r_min; r <= r_max; ++r) {
                for (size_type c = c_min; c <= c_max; ++c) {
                    if (poly.contains(get_point(r, c))) {
                        result.push_back(index(r, c));
                    }
                }
            }

            return result;
        }

      private:
        // Convert world coordinate bounds to grid coordinate bounds
        inline std::tuple<size_type, size_type, size_type, size_type>
        world_to_grid_bounds(const Point &min_world, const Point &max_world) const {
            if (is_identity_) {
                // Fast path for axis-aligned grids
                const double r_min_d = (min_world.y - offset_y_) / inradius_ - 0.5;
                const double r_max_d = (max_world.y - offset_y_) / inradius_ - 0.5;
                const double c_min_d = (min_world.x - offset_x_) / inradius_ - 0.5;
                const double c_max_d = (max_world.x - offset_x_) / inradius_ - 0.5;

                return {static_cast<size_type>(std::max(0.0, std::floor(r_min_d))),
                        static_cast<size_type>(std::min(static_cast<double>(rows_), std::ceil(r_max_d))),
                        static_cast<size_type>(std::max(0.0, std::floor(c_min_d))),
                        static_cast<size_type>(std::min(static_cast<double>(cols_), std::ceil(c_max_d)))};
            } else {
                // Transform AABB corners to grid's local coordinate system
                std::array<Point, 4> world_corners = {
                    Point{min_world.x, min_world.y, 0}, Point{max_world.x, min_world.y, 0},
                    Point{min_world.x, max_world.y, 0}, Point{max_world.x, max_world.y, 0}};

                double min_grid_x = std::numeric_limits<double>::max();
                double max_grid_x = std::numeric_limits<double>::lowest();
                double min_grid_y = std::numeric_limits<double>::max();
                double max_grid_y = std::numeric_limits<double>::lowest();

                // Inverse rotation matrix components
                const double inv_cos = cos_yaw_;  // cos(-theta) = cos(theta)
                const double inv_sin = -sin_yaw_; // sin(-theta) = -sin(theta)

                for (const auto &corner : world_corners) {
                    // Translate to grid origin
                    const double translated_x = corner.x - shift_.point.x;
                    const double translated_y = corner.y - shift_.point.y;

                    // Rotate to grid alignment
                    const double grid_x = translated_x * inv_cos - translated_y * inv_sin;
                    const double grid_y = translated_x * inv_sin + translated_y * inv_cos;

                    min_grid_x = std::min(min_grid_x, grid_x);
                    max_grid_x = std::max(max_grid_x, grid_x);
                    min_grid_y = std::min(min_grid_y, grid_y);
                    max_grid_y = std::max(max_grid_y, grid_y);
                }

                // Convert grid coordinates to indices - rows map to Y, columns map to X
                const double r_min_d = (min_grid_y - offset_y_) / inradius_ - 0.5;
                const double r_max_d = (max_grid_y - offset_y_) / inradius_ - 0.5;
                const double c_min_d = (min_grid_x - offset_x_) / inradius_ - 0.5;
                const double c_max_d = (max_grid_x - offset_x_) / inradius_ - 0.5;

                const double actual_r_min = std::min(r_min_d, r_max_d);
                const double actual_r_max = std::max(r_min_d, r_max_d);
                const double actual_c_min = std::min(c_min_d, c_max_d);
                const double actual_c_max = std::max(c_min_d, c_max_d);

                return {static_cast<size_type>(std::max(0.0, std::floor(actual_r_min))),
                        static_cast<size_type>(std::min(static_cast<double>(rows_), std::ceil(actual_r_max))),
                        static_cast<size_type>(std::max(0.0, std::floor(actual_c_min))),
                        static_cast<size_type>(std::min(static_cast<double>(cols_), std::ceil(actual_c_max)))};
            }
        }

      public:
        // Convert world coordinates to grid indices
        inline std::pair<size_type, size_type> world_to_grid(const Point &world_point) const {
            if (is_identity_) {
                // Fast path for axis-aligned grids
                double col_d = (world_point.x - offset_x_) / inradius_ - 0.5;
                double row_d = (world_point.y - offset_y_) / inradius_ - 0.5;

                // Apply reverse_y transformation if enabled
                if (reverse_y_) {
                    row_d = static_cast<double>(rows_ - 1) - row_d;
                }

                // Clamp to valid range
                size_type col =
                    static_cast<size_type>(std::max(0.0, std::min(static_cast<double>(cols_ - 1), std::round(col_d))));
                size_type row =
                    static_cast<size_type>(std::max(0.0, std::min(static_cast<double>(rows_ - 1), std::round(row_d))));

                return {row, col};
            } else {
                // Transform world point to grid's local coordinate system
                const double translated_x = world_point.x - shift_.point.x;
                const double translated_y = world_point.y - shift_.point.y;

                // Inverse rotation matrix components
                const double inv_cos = cos_yaw_;  // cos(-theta) = cos(theta)
                const double inv_sin = -sin_yaw_; // sin(-theta) = -sin(theta)

                // Rotate to grid alignment
                const double local_x = translated_x * inv_cos - translated_y * inv_sin;
                const double local_y = translated_x * inv_sin + translated_y * inv_cos;

                // Convert to grid coordinates
                double col_d = (local_x + half_width_) / inradius_ - 0.5;
                double row_d = (local_y + half_height_) / inradius_ - 0.5;

                // Apply reverse_y transformation if enabled
                if (reverse_y_) {
                    row_d = static_cast<double>(rows_ - 1) - row_d;
                }

                // Clamp to valid range
                size_type col =
                    static_cast<size_type>(std::max(0.0, std::min(static_cast<double>(cols_ - 1), std::round(col_d))));
                size_type row =
                    static_cast<size_type>(std::max(0.0, std::min(static_cast<double>(rows_ - 1), std::round(row_d))));

                return {row, col};
            }
        }

        // Convert grid indices to world coordinates (alias for get_point for clarity)
        inline Point grid_to_world(size_type row, size_type col) const { return get_point(row, col); }

        // Get value at world coordinate
        inline const T &get_value_at_world(const Point &world_point) const {
            auto [row, col] = world_to_grid(world_point);
            return data_[index(row, col)];
        }

        // Set value at world coordinate
        inline void set_value_at_world(const Point &world_point, const T &value) {
            auto [row, col] = world_to_grid(world_point);
            data_[index(row, col)] = value;
        }

        inline bool operator==(const Grid &other) const {
            return rows_ == other.rows_ && cols_ == other.cols_ && inradius_ == other.inradius_ &&
                   centered_ == other.centered_ && shift_ == other.shift_ && data_ == other.data_;
            // Note: No need to compare precomputed values as they're derived from other fields
        }

        inline bool operator!=(const Grid &other) const { return !(*this == other); }
    };

} // namespace concord
