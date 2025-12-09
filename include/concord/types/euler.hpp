#pragma once

#include "../math.hpp"
#include <cmath>

namespace concord {

    struct Quaternion; // forward declaration

    struct Euler {
        double roll = 0.0;  // rotation about x-axis
        double pitch = 0.0; // rotation about y-axis
        double yaw = 0.0;   // rotation about z-axis

        Euler() = default;
        Euler(double roll_, double pitch_, double yaw_) : roll(roll_), pitch(pitch_), yaw(yaw_) {}
        explicit Euler(const Quaternion &q) noexcept;
        inline bool is_set() const { return roll != 0.0 || pitch != 0.0 || yaw != 0.0; }

        inline double yaw_cos() const { return std::cos(yaw * 0.5); }
        inline double yaw_sin() const { return std::sin(yaw * 0.5); }

        // Mathematical operations
        inline Euler operator+(const Euler &other) const {
            return Euler{roll + other.roll, pitch + other.pitch, yaw + other.yaw};
        }
        inline Euler operator-(const Euler &other) const {
            return Euler{roll - other.roll, pitch - other.pitch, yaw - other.yaw};
        }
        inline Euler operator*(double scale) const { return Euler{roll * scale, pitch * scale, yaw * scale}; }

        inline bool operator==(const Euler &other) const {
            return roll == other.roll && pitch == other.pitch && yaw == other.yaw;
        }

        inline bool operator!=(const Euler &other) const { return !(*this == other); }

        // Angle normalization
        inline Euler normalized() const {
            auto normalize_angle = [](double angle) {
                while (angle > M_PI)
                    angle -= 2.0 * M_PI;
                while (angle < -M_PI)
                    angle += 2.0 * M_PI;
                return angle;
            };
            return Euler{normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)};
        }

        // Convert to rotation matrix (3x3)
        inline Mat3d to_rotation_matrix() const {
            double cr = std::cos(roll), sr = std::sin(roll);
            double cp = std::cos(pitch), sp = std::sin(pitch);
            double cy = std::cos(yaw), sy = std::sin(yaw);

            Mat3d R;
            R[0][0] = cy * cp;
            R[0][1] = cy * sp * sr - sy * cr;
            R[0][2] = cy * sp * cr + sy * sr;
            R[1][0] = sy * cp;
            R[1][1] = sy * sp * sr + cy * cr;
            R[1][2] = sy * sp * cr - cy * sr;
            R[2][0] = -sp;
            R[2][1] = cp * sr;
            R[2][2] = cp * cr;
            return R;
        }

        // Rotate a vector
        inline Vec3d rotate(const Vec3d &v) const {
            Mat3d R = to_rotation_matrix();
            return R * v;
        }
    };

} // namespace concord
