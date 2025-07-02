#pragma once

#include "../errors/error_handling.hpp"
#include "../math/math.hpp"
#include "euler.hpp"
#include <cmath>

namespace concord {

    struct Quaternion {
        double w = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        Quaternion() = default;
        Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
        explicit Quaternion(const Euler &e) noexcept;
        inline bool is_set() const { return w != 0.0 || x != 0.0 || y != 0.0 || z != 0.0; }

        // Mathematical operations
        inline Quaternion operator+(const Quaternion &other) const {
            return Quaternion{w + other.w, x + other.x, y + other.y, z + other.z};
        }
        inline Quaternion operator-(const Quaternion &other) const {
            return Quaternion{w - other.w, x - other.x, y - other.y, z - other.z};
        }
        inline Quaternion operator*(const Quaternion &other) const {
            return Quaternion{w * other.w - x * other.x - y * other.y - z * other.z,
                              w * other.x + x * other.w + y * other.z - z * other.y,
                              w * other.y - x * other.z + y * other.w + z * other.x,
                              w * other.z + x * other.y - y * other.x + z * other.w};
        }
        inline Quaternion operator*(double scale) const {
            return Quaternion{w * scale, x * scale, y * scale, z * scale};
        }

        inline bool operator==(const Quaternion& other) const {
            return w == other.w && x == other.x && y == other.y && z == other.z;
        }

        inline bool operator!=(const Quaternion& other) const {
            return !(*this == other);
        }

        // Quaternion operations
        inline double norm() const {
            validation::validate_finite(w, "quaternion w");
            validation::validate_finite(x, "quaternion x");
            validation::validate_finite(y, "quaternion y");
            validation::validate_finite(z, "quaternion z");
            return std::sqrt(w * w + x * x + y * y + z * z);
        }

        inline Quaternion normalized() const {
            double n = norm();
            if (n < 1e-15) {
                throw MathematicalException("cannot normalize zero quaternion");
            }
            return (*this) * (1.0 / n);
        }

        inline Quaternion conjugate() const { return Quaternion{w, -x, -y, -z}; }

        inline Quaternion inverse() const {
            double n2 = w * w + x * x + y * y + z * z;
            if (n2 < 1e-15) {
                throw MathematicalException("cannot invert zero quaternion");
            }
            return conjugate() * (1.0 / n2);
        }

        // Rotation operations
        inline Vec3d rotate(const Vec3d &v) const {
            Quaternion qv{0, v[0], v[1], v[2]};
            Quaternion result = (*this) * qv * conjugate();
            return Vec3d{result.x, result.y, result.z};
        }

        // SLERP interpolation
        inline static Quaternion slerp(const Quaternion &q1, const Quaternion &q2, double t) {
            double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
            if (dot < 0.0) {
                return slerp(q1, Quaternion{-q2.w, -q2.x, -q2.y, -q2.z}, t);
            }
            if (dot > 0.9995) {
                // Linear interpolation for very close quaternions
                Quaternion result = q1 + (q2 - q1) * t;
                return result.normalized();
            }
            double theta = std::acos(std::abs(dot));
            double sin_theta = std::sin(theta);
            double t1 = std::sin((1.0 - t) * theta) / sin_theta;
            double t2 = std::sin(t * theta) / sin_theta;
            return q1 * t1 + q2 * t2;
        }
    };

    // Cross-dependency implementations
    inline Euler::Euler(const Quaternion &q) noexcept {
        // roll (x-axis rotation)
        double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1.0) {
            pitch = std::copysign(M_PI / 2.0, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    inline Quaternion::Quaternion(const Euler &e) noexcept {
        double cy = std::cos(e.yaw * 0.5);
        double sy = std::sin(e.yaw * 0.5);
        double cp = std::cos(e.pitch * 0.5);
        double sp = std::sin(e.pitch * 0.5);
        double cr = std::cos(e.roll * 0.5);
        double sr = std::sin(e.roll * 0.5);

        w = cr * cp * cy + sr * sp * sy;
        x = sr * cp * cy - cr * sp * sy;
        y = cr * sp * cy + sr * cp * sy;
        z = cr * cp * sy - sr * sp * cy;
    }

} // namespace concord
