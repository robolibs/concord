#pragma once

#include "../../core/types.hpp"
#include "../polygon/polygon.hpp"
#include <cassert>
#include <cmath>
#include <cstddef>
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

      public:
        Grid() = default;

        Grid(size_type rows, size_type cols, double diameter, bool centered, concord::Pose shift)
            : rows_{rows}, cols_{cols}, inradius_{diameter}, centered_{centered}, shift_{shift}, data_(rows * cols) {
            // Precompute transform constants
            height_ = centered_ ? cols_ * inradius_ : 0.0;
            width_ = centered_ ? rows_ * inradius_ : 0.0;
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

            // Initialize data with default T values
            std::fill(data_.begin(), data_.end(), T{});
        }

        Grid(const Polygon &poly, double resolution, bool centered = true)
            : Grid(static_cast<size_type>(std::ceil(poly.get_obb().size.x / resolution)),
                   static_cast<size_type>(std::ceil(poly.get_obb().size.y / resolution)), resolution, centered,
                   poly.get_obb().pose) {}

        // Compute point on-demand (optimized with fast paths)
        [[gnu::hot]] inline Point get_point(size_type r, size_type c) const {
            if (r >= rows_ || c >= cols_) {
                throw std::out_of_range("Grid indices out of bounds");
            }

            // Fast path for identity transform (no rotation, no translation)
            if (is_identity_) [[likely]] {
                return Point{(static_cast<double>(r) + 0.5) * inradius_ + offset_x_,
                             (static_cast<double>(c) + 0.5) * inradius_ + offset_y_, 0.0};
            }

            // General case with rotation/translation
            double local_x = (static_cast<double>(r) + 0.5) * inradius_ - half_width_;
            double local_y = (static_cast<double>(c) + 0.5) * inradius_ - half_height_;

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

        inline std::span<T> row(size_type r) noexcept { return {&data_[index(r, 0)], cols_}; }

        inline std::span<const T> row(size_type r) const noexcept { return {&data_[index(r, 0)], cols_}; }

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

        // SIMD-optimized flatten_points with memory prefetching for bulk point generation
        inline std::vector<std::array<float, 3>> flatten_points() const {
            std::vector<std::array<float, 3>> points;
            points.reserve(rows_ * cols_);

            // Only use prefetching for large grids to avoid cache pollution
            const bool use_prefetch = (rows_ * cols_) > 10000; // ~40KB threshold
            const size_t prefetch_distance = 2;                // Prefetch 2 rows ahead

            if (is_identity_) [[likely]] {
                // Vectorized identity transform path with prefetching
                const float inradius_f = static_cast<float>(inradius_);
                const float offset_x_f = static_cast<float>(offset_x_);
                const float offset_y_f = static_cast<float>(offset_y_);

                for (size_type r = 0; r < rows_; ++r) {
                    // Prefetch future data to reduce memory latency
                    if (use_prefetch && (r + prefetch_distance) < rows_) [[likely]] {
                        // Prefetch next rows of output data (write hint)
                        const size_t prefetch_idx = (r + prefetch_distance) * cols_;
                        if (prefetch_idx < points.capacity()) {
                            __builtin_prefetch(&points[0] + prefetch_idx, 1, 1); // Write, low temporal locality
                        }
                    }

                    const float base_x = (static_cast<float>(r) + 0.5f) * inradius_f + offset_x_f;
                    for (size_type c = 0; c < cols_; ++c) {
                        const float y = (static_cast<float>(c) + 0.5f) * inradius_f + offset_y_f;
                        points.push_back({base_x, y, 0.0f});
                    }
                }
            } else {
                // General case with prefetching
                const float inradius_f = static_cast<float>(inradius_);
                const float half_width_f = static_cast<float>(half_width_);
                const float half_height_f = static_cast<float>(half_height_);
                const float cos_yaw_f = static_cast<float>(cos_yaw_);
                const float sin_yaw_f = static_cast<float>(sin_yaw_);
                const float shift_x_f = static_cast<float>(shift_.point.x);
                const float shift_y_f = static_cast<float>(shift_.point.y);

                for (size_type r = 0; r < rows_; ++r) {
                    // Prefetch future computations
                    if (use_prefetch && (r + prefetch_distance) < rows_) [[likely]] {
                        const size_t prefetch_idx = (r + prefetch_distance) * cols_;
                        if (prefetch_idx < points.capacity()) {
                            __builtin_prefetch(&points[0] + prefetch_idx, 1, 1); // Write prefetch
                        }
                    }

                    const float local_x_base = (static_cast<float>(r) + 0.5f) * inradius_f - half_width_f;
                    for (size_type c = 0; c < cols_; ++c) {
                        const float local_y = (static_cast<float>(c) + 0.5f) * inradius_f - half_height_f;

                        const float world_x = cos_yaw_f * local_x_base - sin_yaw_f * local_y + shift_x_f;
                        const float world_y = sin_yaw_f * local_x_base + cos_yaw_f * local_y + shift_y_f;

                        points.push_back({world_x, world_y, 0.0f});
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

        // Optimized indices_within with prefetching for polygon intersection tests
        inline std::vector<size_type> indices_within(const Polygon &poly) const {
            if (!poly.isConnected()) {
                throw std::invalid_argument("Polygon must have at least 3 vertices");
            }
            std::vector<size_type> out;
            out.reserve(rows_ * cols_);

            // Use prefetching for large grids
            const bool use_prefetch = (rows_ * cols_) > 10000;
            const size_t prefetch_distance = 3; // Prefetch 3 rows ahead for polygon tests

            for (size_type r = 0; r < rows_; ++r) {
                // Prefetch future polygon computation data
                if (use_prefetch && (r + prefetch_distance) < rows_) [[likely]] {
                    // Prefetch output vector memory
                    const size_t current_size = out.size();
                    const size_t estimated_future_size =
                        current_size + (prefetch_distance * cols_ / 4); // Estimate ~25% hit rate
                    if (estimated_future_size < out.capacity()) {
                        __builtin_prefetch(&out[0] + estimated_future_size, 1, 2); // Write prefetch, moderate locality
                    }
                }

                for (size_type c = 0; c < cols_; ++c) {
                    // Prefetch next few points for get_point() calls
                    if (use_prefetch && c + 8 < cols_) [[likely]] {
                        // Hint for upcoming point calculations (read-only, high temporal locality)
                        __builtin_prefetch(static_cast<const void *>(&rows_), 0, 3);
                    }

                    if (poly.contains(get_point(r, c))) {
                        out.push_back(index(r, c));
                    }
                }
            }
            return out;
        }

        inline bool operator==(const Grid &other) const {
            return rows_ == other.rows_ && cols_ == other.cols_ && inradius_ == other.inradius_ &&
                   centered_ == other.centered_ && shift_ == other.shift_ && data_ == other.data_;
            // Note: No need to compare precomputed values as they're derived from other fields
        }

        inline bool operator!=(const Grid &other) const { return !(*this == other); }
    };

} // namespace concord
