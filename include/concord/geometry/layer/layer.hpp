#pragma once

#include "../../core/types.hpp"
#include "../polygon/polygon.hpp"
#include "../grid/grid.hpp"
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

    template <typename T> class Layer {
        using size_type = std::size_t;
        using value_type = T;
        using const_reference = const T &;
        using reference = T &;

      private:
        size_type rows_{0};
        size_type cols_{0};
        size_type layers_{0};
        double inradius_{0.0};        // Cell size in XY plane
        double layer_height_{0.0};    // Cell size in Z direction
        bool centered_{false};
        bool reverse_y_{false};
        bool reverse_z_{false};
        concord::Pose shift_{};
        std::vector<T> data_;

        // Precomputed transform constants
        double width_{0.0};
        double height_{0.0};
        double depth_{0.0};
        double cos_yaw_{1.0};
        double sin_yaw_{0.0};
        double cos_pitch_{1.0};
        double sin_pitch_{0.0};
        double cos_roll_{1.0};
        double sin_roll_{0.0};
        double half_width_{0.0};
        double half_height_{0.0};
        double half_depth_{0.0};
        double offset_x_{0.0};
        double offset_y_{0.0};
        double offset_z_{0.0};
        bool is_identity_{true};

        // Precomputed local coordinates
        std::vector<double> x_coords_;
        std::vector<double> y_coords_;
        std::vector<double> z_coords_;

      public:
        Layer() = default;

        Layer(size_type rows, size_type cols, size_type layers, double radius, double layer_height, bool centered,
              concord::Pose shift, bool reverse_y = false, bool reverse_z = false)
            : rows_{rows}, cols_{cols}, layers_{layers}, inradius_{radius}, layer_height_{layer_height},
              centered_{centered}, shift_{shift}, reverse_y_{reverse_y}, reverse_z_{reverse_z},
              data_(rows * cols * layers) {

            if (rows == 0 || cols == 0 || layers == 0) {
                throw std::invalid_argument("Layer dimensions must be positive");
            }
            if (radius <= 0.0 || layer_height <= 0.0) {
                throw std::invalid_argument("Layer cell sizes must be positive");
            }

            // Precompute transform constants
            width_ = centered_ ? cols_ * inradius_ : 0.0;
            height_ = centered_ ? rows_ * inradius_ : 0.0;
            depth_ = centered_ ? layers_ * layer_height_ : 0.0;

            // 3D rotation precomputation (using Euler angles from Pose)
            cos_yaw_ = std::cos(shift_.angle.yaw);
            sin_yaw_ = std::sin(shift_.angle.yaw);
            cos_pitch_ = std::cos(shift_.angle.pitch);
            sin_pitch_ = std::sin(shift_.angle.pitch);
            cos_roll_ = std::cos(shift_.angle.roll);
            sin_roll_ = std::sin(shift_.angle.roll);

            half_width_ = width_ * 0.5;
            half_height_ = height_ * 0.5;
            half_depth_ = depth_ * 0.5;

            // Check for identity transform
            constexpr double epsilon = 1e-15;
            is_identity_ = (std::abs(cos_yaw_ - 1.0) < epsilon) && (std::abs(sin_yaw_) < epsilon) &&
                           (std::abs(cos_pitch_ - 1.0) < epsilon) && (std::abs(sin_pitch_) < epsilon) &&
                           (std::abs(cos_roll_ - 1.0) < epsilon) && (std::abs(sin_roll_) < epsilon) &&
                           (std::abs(shift_.point.x) < epsilon) && (std::abs(shift_.point.y) < epsilon) &&
                           (std::abs(shift_.point.z) < epsilon);

            offset_x_ = -half_width_ + shift_.point.x;
            offset_y_ = -half_height_ + shift_.point.y;
            offset_z_ = centered_ ? -half_depth_ + shift_.point.z : shift_.point.z;

            // Precompute local coordinates
            x_coords_.resize(cols_);
            y_coords_.resize(rows_);
            z_coords_.resize(layers_);

            for (size_type c = 0; c < cols_; ++c) {
                x_coords_[c] = (static_cast<double>(c) + 0.5) * inradius_;
            }

            for (size_type r = 0; r < rows_; ++r) {
                if (reverse_y_) {
                    y_coords_[r] = (static_cast<double>(rows_ - 1 - r) + 0.5) * inradius_;
                } else {
                    y_coords_[r] = (static_cast<double>(r) + 0.5) * inradius_;
                }
            }

            for (size_type l = 0; l < layers_; ++l) {
                if (reverse_z_) {
                    z_coords_[l] = (static_cast<double>(layers_ - 1 - l) + 0.5) * layer_height_;
                } else {
                    z_coords_[l] = (static_cast<double>(l) + 0.5) * layer_height_;
                }
                if (centered_) {
                    z_coords_[l] += layer_height_ * 0.5;
                }
            }

            std::fill(data_.begin(), data_.end(), T{});
        }

        // 3D indexing using depth-major order: layer changes fastest for better memory locality
        constexpr inline size_type index(size_type r, size_type c, size_type l) const noexcept {
            return (r * cols_ + c) * layers_ + l;
        }

        inline size_type index_checked(size_type r, size_type c, size_type l) const {
            if (r >= rows_ || c >= cols_ || l >= layers_) {
                throw std::out_of_range("Layer indices out of bounds");
            }
            return index(r, c, l);
        }

        // 3D point computation with optimized identity transform path
        [[gnu::hot]] inline Point get_point(size_type r, size_type c, size_type l) const {
            if (r >= rows_ || c >= cols_ || l >= layers_) {
                throw std::out_of_range("Layer indices out of bounds");
            }

            // Fast path for identity transform
            if (is_identity_) [[likely]] {
                return Point{x_coords_[c] + offset_x_, y_coords_[r] + offset_y_, z_coords_[l] + offset_z_};
            }

            // General case with 3D rotation
            double local_x = x_coords_[c] - half_width_;
            double local_y = y_coords_[r] - half_height_;
            double local_z = z_coords_[l] - half_depth_;

            // Apply 3D rotation (ZYX Euler angles: Yaw, Pitch, Roll)
            double temp_x = local_x;
            double temp_y = local_y;
            double temp_z = local_z;

            // Yaw rotation (around Z-axis)
            double x_after_yaw = cos_yaw_ * temp_x - sin_yaw_ * temp_y;
            double y_after_yaw = sin_yaw_ * temp_x + cos_yaw_ * temp_y;

            // Pitch rotation (around Y-axis)
            double x_after_pitch = cos_pitch_ * x_after_yaw + sin_pitch_ * temp_z;
            double z_after_pitch = -sin_pitch_ * x_after_yaw + cos_pitch_ * temp_z;

            // Roll rotation (around X-axis)
            double y_final = cos_roll_ * y_after_yaw - sin_roll_ * z_after_pitch;
            double z_final = sin_roll_ * y_after_yaw + cos_roll_ * z_after_pitch;

            return Point{x_after_pitch + shift_.point.x, y_final + shift_.point.y, z_final + shift_.point.z};
        }

        // Data access methods
        inline void set_value(size_type r, size_type c, size_type l, const T &value) {
            data_[index_checked(r, c, l)] = value;
        }

        inline reference operator()(size_type r, size_type c, size_type l) noexcept { return data_[index(r, c, l)]; }

        inline const_reference operator()(size_type r, size_type c, size_type l) const noexcept {
            return data_[index(r, c, l)];
        }

        inline reference at(size_type r, size_type c, size_type l) { return data_.at(index_checked(r, c, l)); }

        inline const_reference at(size_type r, size_type c, size_type l) const {
            return data_.at(index_checked(r, c, l));
        }

        // Layer access - return span of a complete 2D layer
        // Note: This returns a non-contiguous view, so we can't use std::span
        // Instead, we provide a custom view or copy the data
        inline std::vector<T> layer(size_type l) {
            if (l >= layers_) {
                throw std::out_of_range("Layer index out of bounds");
            }
            std::vector<T> result;
            result.reserve(rows_ * cols_);
            for (size_type r = 0; r < rows_; ++r) {
                for (size_type c = 0; c < cols_; ++c) {
                    result.push_back(data_[index(r, c, l)]);
                }
            }
            return result;
        }

        inline std::vector<T> layer(size_type l) const {
            if (l >= layers_) {
                throw std::out_of_range("Layer index out of bounds");
            }
            std::vector<T> result;
            result.reserve(rows_ * cols_);
            for (size_type r = 0; r < rows_; ++r) {
                for (size_type c = 0; c < cols_; ++c) {
                    result.push_back(data_[index(r, c, l)]);
                }
            }
            return result;
        }

        // Row access within a specific layer
        // Note: With depth-major storage, row data is not contiguous
        inline std::vector<T> row(size_type r, size_type l) {
            if (r >= rows_ || l >= layers_) {
                throw std::out_of_range("Row or layer index out of bounds");
            }
            std::vector<T> result;
            result.reserve(cols_);
            for (size_type c = 0; c < cols_; ++c) {
                result.push_back(data_[index(r, c, l)]);
            }
            return result;
        }

        inline std::vector<T> row(size_type r, size_type l) const {
            if (r >= rows_ || l >= layers_) {
                throw std::out_of_range("Row or layer index out of bounds");
            }
            std::vector<T> result;
            result.reserve(cols_);
            for (size_type c = 0; c < cols_; ++c) {
                result.push_back(data_[index(r, c, l)]);
            }
            return result;
        }

        // Accessors
        constexpr size_type rows() const noexcept { return rows_; }
        constexpr size_type cols() const noexcept { return cols_; }
        constexpr size_type layers() const noexcept { return layers_; }
        constexpr double inradius() const noexcept { return inradius_; }
        constexpr double layer_height() const noexcept { return layer_height_; }
        constexpr bool centered() const noexcept { return centered_; }
        constexpr bool reverse_y() const noexcept { return reverse_y_; }
        constexpr bool reverse_z() const noexcept { return reverse_z_; }
        const concord::Pose& pose() const noexcept { return shift_; }
        const concord::Pose& shift() const noexcept { return shift_; } // Alias for backward compatibility

        // Data iterators
        inline auto begin() noexcept { return data_.begin(); }
        inline auto end() noexcept { return data_.end(); }
        inline auto begin() const noexcept { return data_.begin(); }
        inline auto end() const noexcept { return data_.end(); }

        // Optimized bulk point generation with 4D output (x, y, z, data)
        inline std::vector<std::array<float, 4>> flatten_points_with_data() const {
            std::vector<std::array<float, 4>> points;
            const size_type total_points = rows_ * cols_ * layers_;
            points.resize(total_points);

            if (is_identity_) [[likely]] {
                const float inradius_f = static_cast<float>(inradius_);
                const float layer_height_f = static_cast<float>(layer_height_);
                const float offset_x_f = static_cast<float>(offset_x_);
                const float offset_y_f = static_cast<float>(offset_y_);
                const float offset_z_f = static_cast<float>(offset_z_);

                size_type idx = 0;
                for (size_type r = 0; r < rows_; ++r) {
                    const float base_y = (static_cast<float>(r) + 0.5f) * inradius_f + offset_y_f;
                    for (size_type c = 0; c < cols_; ++c) {
                        const float base_x = (static_cast<float>(c) + 0.5f) * inradius_f + offset_x_f;
                        for (size_type l = 0; l < layers_; ++l) {
                            const float z = (static_cast<float>(l) + 0.5f) * layer_height_f + offset_z_f;
                            const float data_value = static_cast<float>(data_[index(r, c, l)]);
                            points[idx++] = {base_x, base_y, z, data_value};
                        }
                    }
                }
            } else {
                // General case with 3D rotation - more complex but maintains performance
                size_type idx = 0;
                for (size_type r = 0; r < rows_; ++r) {
                    for (size_type c = 0; c < cols_; ++c) {
                        for (size_type l = 0; l < layers_; ++l) {
                            Point world_point = get_point(r, c, l);
                            const float data_value = static_cast<float>(data_[index(r, c, l)]);
                            points[idx++] = {static_cast<float>(world_point.x), static_cast<float>(world_point.y),
                                             static_cast<float>(world_point.z), data_value};
                        }
                    }
                }
            }
            return points;
        }

        // 3D coordinate transformation: world to grid indices
        inline std::tuple<size_type, size_type, size_type> world_to_grid(const Point &world_point) const {
            if (is_identity_) {
                double col_d = (world_point.x - offset_x_) / inradius_ - 0.5;
                double row_d = (world_point.y - offset_y_) / inradius_ - 0.5;
                double layer_d = (world_point.z - offset_z_) / layer_height_ - 0.5;
                if (centered_) {
                    layer_d -= 0.5;  // Account for centering offset added to z_coords_
                }

                if (reverse_y_) {
                    row_d = static_cast<double>(rows_ - 1) - row_d;
                }
                if (reverse_z_) {
                    layer_d = static_cast<double>(layers_ - 1) - layer_d;
                }

                size_type col = static_cast<size_type>(
                    std::max(0.0, std::min(static_cast<double>(cols_ - 1), std::round(col_d))));
                size_type row = static_cast<size_type>(
                    std::max(0.0, std::min(static_cast<double>(rows_ - 1), std::round(row_d))));
                size_type layer = static_cast<size_type>(
                    std::max(0.0, std::min(static_cast<double>(layers_ - 1), std::round(layer_d))));

                return {row, col, layer};
            } else {
                // Inverse 3D transformation
                const double translated_x = world_point.x - shift_.point.x;
                const double translated_y = world_point.y - shift_.point.y;
                const double translated_z = world_point.z - shift_.point.z;

                // Inverse 3D rotation (reverse order: Roll, Pitch, Yaw)
                // Roll inverse (around X-axis)
                double y_after_roll_inv = cos_roll_ * translated_y + sin_roll_ * translated_z;
                double z_after_roll_inv = -sin_roll_ * translated_y + cos_roll_ * translated_z;

                // Pitch inverse (around Y-axis)
                double x_after_pitch_inv = cos_pitch_ * translated_x - sin_pitch_ * z_after_roll_inv;
                double z_after_pitch_inv = sin_pitch_ * translated_x + cos_pitch_ * z_after_roll_inv;

                // Yaw inverse (around Z-axis)
                double local_x = cos_yaw_ * x_after_pitch_inv + sin_yaw_ * y_after_roll_inv;
                double local_y = -sin_yaw_ * x_after_pitch_inv + cos_yaw_ * y_after_roll_inv;
                double local_z = z_after_pitch_inv;

                double col_d = (local_x + half_width_) / inradius_ - 0.5;
                double row_d = (local_y + half_height_) / inradius_ - 0.5;
                double layer_d = (local_z + half_depth_) / layer_height_ - 0.5;
                if (centered_) {
                    layer_d -= 0.5;  // Account for centering offset added to z_coords_
                }

                if (reverse_y_) {
                    row_d = static_cast<double>(rows_ - 1) - row_d;
                }
                if (reverse_z_) {
                    layer_d = static_cast<double>(layers_ - 1) - layer_d;
                }

                size_type col = static_cast<size_type>(
                    std::max(0.0, std::min(static_cast<double>(cols_ - 1), std::round(col_d))));
                size_type row = static_cast<size_type>(
                    std::max(0.0, std::min(static_cast<double>(rows_ - 1), std::round(row_d))));
                size_type layer = static_cast<size_type>(
                    std::max(0.0, std::min(static_cast<double>(layers_ - 1), std::round(layer_d))));

                return {row, col, layer};
            }
        }

        // Convenience methods
        inline Point grid_to_world(size_type row, size_type col, size_type layer) const {
            return get_point(row, col, layer);
        }

        inline const T &get_value_at_world(const Point &world_point) const {
            auto [row, col, layer] = world_to_grid(world_point);
            return data_[index(row, col, layer)];
        }

        inline void set_value_at_world(const Point &world_point, const T &value) {
            auto [row, col, layer] = world_to_grid(world_point);
            data_[index(row, col, layer)] = value;
        }

        // Equality operators
        inline bool operator==(const Layer &other) const {
            return rows_ == other.rows_ && cols_ == other.cols_ && layers_ == other.layers_ &&
                   inradius_ == other.inradius_ && layer_height_ == other.layer_height_ &&
                   centered_ == other.centered_ && shift_ == other.shift_ && reverse_y_ == other.reverse_y_ &&
                   reverse_z_ == other.reverse_z_ && data_ == other.data_;
        }

        inline bool operator!=(const Layer &other) const { return !(*this == other); }

        // Extract 2D Grid from specific layer (creates a copy for compatibility)
        Grid<T> extract_grid(size_type layer_idx) const {
            if (layer_idx >= layers_) {
                throw std::out_of_range("Layer index out of bounds");
            }

            // Create a 2D pose from the 3D pose (ignoring Z components)
            concord::Pose grid_pose = shift_;
            grid_pose.point.z = 0.0; // 2D grids don't use Z
            grid_pose.angle.pitch = 0.0;
            grid_pose.angle.roll = 0.0;

            Grid<T> grid(rows_, cols_, inradius_, centered_, grid_pose, reverse_y_);

            // Copy data from the specified layer
            for (size_type r = 0; r < rows_; ++r) {
                for (size_type c = 0; c < cols_; ++c) {
                    grid(r, c) = (*this)(r, c, layer_idx);
                }
            }

            return grid;
        }

        // Get 8 corner points of the 3D layer
        inline std::array<Point, 8> corners() const {
            if (rows_ == 0 || cols_ == 0 || layers_ == 0) {
                throw std::runtime_error("Layer is empty; cannot get corners");
            }

            const size_type r0 = 0, r1 = rows_ - 1;
            const size_type c0 = 0, c1 = cols_ - 1;
            const size_type l0 = 0, l1 = layers_ - 1;

            return {get_point(r0, c0, l0), // front-bottom-left
                    get_point(r0, c1, l0), // front-bottom-right
                    get_point(r1, c1, l0), // back-bottom-right
                    get_point(r1, c0, l0), // back-bottom-left
                    get_point(r0, c0, l1), // front-top-left
                    get_point(r0, c1, l1), // front-top-right
                    get_point(r1, c1, l1), // back-top-right
                    get_point(r1, c0, l1)  // back-top-left
            };
        }
    };

} // namespace concord