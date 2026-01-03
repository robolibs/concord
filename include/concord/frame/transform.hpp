#pragma once

#include <cmath>
#include <datapod/datapod.hpp>
#include <ostream>

#include "../earth/local_axes.hpp"
#include "tags.hpp"

namespace concord::frame {

    // ============================================================================
    // Rotation<ToFrame, FromFrame> - SO(3) rotation between frames
    // ============================================================================
    template <typename ToFrame, typename FromFrame, typename T = double> struct Rotation {
        dp::Quaternion q{1.0, 0.0, 0.0, 0.0}; // w,x,y,z - identity

        // Constructors
        Rotation() = default;
        explicit Rotation(const dp::Quaternion &quat) : q(quat.normalized()) {}

        // From rotation matrix
        static Rotation from_matrix(const dp::mat::Matrix3x3d &R) {
            dp::Quaternion quat;
            const T trace = R(0, 0) + R(1, 1) + R(2, 2);

            if (trace > 0) {
                const T s = 0.5 / std::sqrt(trace + 1.0);
                quat =
                    dp::Quaternion{0.25 / s, (R(2, 1) - R(1, 2)) * s, (R(0, 2) - R(2, 0)) * s, (R(1, 0) - R(0, 1)) * s};
            } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
                const T s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
                quat =
                    dp::Quaternion{(R(2, 1) - R(1, 2)) / s, 0.25 * s, (R(0, 1) + R(1, 0)) / s, (R(0, 2) + R(2, 0)) / s};
            } else if (R(1, 1) > R(2, 2)) {
                const T s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
                quat =
                    dp::Quaternion{(R(0, 2) - R(2, 0)) / s, (R(0, 1) + R(1, 0)) / s, 0.25 * s, (R(1, 2) + R(2, 1)) / s};
            } else {
                const T s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
                quat =
                    dp::Quaternion{(R(1, 0) - R(0, 1)) / s, (R(0, 2) + R(2, 0)) / s, (R(1, 2) + R(2, 1)) / s, 0.25 * s};
            }
            return Rotation{quat};
        }

        // From axis-angle representation
        static Rotation from_axis_angle(const dp::Point &axis, T angle_rad) {
            const T half = angle_rad * 0.5;
            const T s = std::sin(half);
            const T c = std::cos(half);
            const T mag = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
            if (mag < 1e-10) {
                return Rotation{}; // identity
            }
            return Rotation{dp::Quaternion{c, axis.x / mag * s, axis.y / mag * s, axis.z / mag * s}};
        }

        // From Euler angles (ZYX convention: yaw, pitch, roll)
        static Rotation from_euler_zyx(T yaw_rad, T pitch_rad, T roll_rad) {
            const T cy = std::cos(yaw_rad * 0.5), sy = std::sin(yaw_rad * 0.5);
            const T cp = std::cos(pitch_rad * 0.5), sp = std::sin(pitch_rad * 0.5);
            const T cr = std::cos(roll_rad * 0.5), sr = std::sin(roll_rad * 0.5);

            return Rotation{dp::Quaternion{cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                                           cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy}};
        }

        // Identity rotation
        static Rotation identity() { return Rotation{}; }

        // Inverse rotation (swaps To/From)
        Rotation<FromFrame, ToFrame, T> inverse() const { return Rotation<FromFrame, ToFrame, T>{q.conjugate()}; }

        // Apply rotation to a point (quaternion rotation: q * p * q^-1)
        dp::Point apply(const dp::Point &p) const {
            // Convert point to quaternion (0, px, py, pz)
            dp::Quaternion p_quat{0.0, p.x, p.y, p.z};
            // Rotate: q * p * q^-1 (for unit quaternion, q^-1 = conjugate)
            dp::Quaternion result = q * p_quat * q.conjugate();
            return dp::Point{result.x, result.y, result.z};
        }

        // Convert to rotation matrix
        dp::mat::Matrix3x3d to_matrix() const {
            dp::mat::Matrix3x3d R{};
            const T w = q.w, x = q.x, y = q.y, z = q.z;
            const T xx = x * x, yy = y * y, zz = z * z;
            const T xy = x * y, xz = x * z, yz = y * z;
            const T wx = w * x, wy = w * y, wz = w * z;

            R(0, 0) = 1.0 - 2.0 * (yy + zz);
            R(0, 1) = 2.0 * (xy - wz);
            R(0, 2) = 2.0 * (xz + wy);
            R(1, 0) = 2.0 * (xy + wz);
            R(1, 1) = 1.0 - 2.0 * (xx + zz);
            R(1, 2) = 2.0 * (yz - wx);
            R(2, 0) = 2.0 * (xz - wy);
            R(2, 1) = 2.0 * (yz + wx);
            R(2, 2) = 1.0 - 2.0 * (xx + yy);
            return R;
        }

        // Get quaternion
        const dp::Quaternion &quaternion() const { return q; }
    };

    // Rotation composition: R_AC = R_AB * R_BC
    template <typename A, typename B, typename C, typename T>
    Rotation<A, C, T> operator*(const Rotation<A, B, T> &r_ab, const Rotation<B, C, T> &r_bc) {
        return Rotation<A, C, T>{r_ab.q * r_bc.q};
    }

    // Apply rotation to point: R * p
    template <typename To, typename From, typename T>
    dp::Point operator*(const Rotation<To, From, T> &r, const dp::Point &p) {
        return r.apply(p);
    }

    template <typename To, typename From, typename T>
    std::ostream &operator<<(std::ostream &os, const Rotation<To, From, T> &r) {
        return os << "Rotation{q=[" << r.q.w << "," << r.q.x << "," << r.q.y << "," << r.q.z << "]}";
    }

    // ============================================================================
    // Transform<ToFrame, FromFrame> - SE(3) rigid body transform
    // ============================================================================
    template <typename ToFrame, typename FromFrame, typename T = double> struct Transform {
        Rotation<ToFrame, FromFrame, T> rotation{};
        dp::Point translation{0.0, 0.0, 0.0}; // expressed in ToFrame

        // Constructors
        Transform() = default;
        Transform(const Rotation<ToFrame, FromFrame, T> &r, const dp::Point &t) : rotation(r), translation(t) {}

        // From quaternion and translation
        static Transform from_qt(const dp::Quaternion &q, const dp::Point &t) {
            return Transform{Rotation<ToFrame, FromFrame, T>{q}, t};
        }

        // From rotation matrix and translation
        static Transform from_Rt(const dp::mat::Matrix3x3d &R, const dp::Point &t) {
            return Transform{Rotation<ToFrame, FromFrame, T>::from_matrix(R), t};
        }

        // Identity transform
        static Transform identity() { return Transform{}; }

        // Inverse transform: T_BA from T_AB
        Transform<FromFrame, ToFrame, T> inverse() const {
            auto r_inv = rotation.inverse();
            dp::Point t_inv = r_inv.apply(translation);
            return Transform<FromFrame, ToFrame, T>{r_inv, dp::Point{-t_inv.x, -t_inv.y, -t_inv.z}};
        }

        // Apply transform to point: p_to = R * p_from + t
        dp::Point apply(const dp::Point &p_from) const { return rotation.apply(p_from) + translation; }

        // Get rotation matrix
        dp::mat::Matrix3x3d rotation_matrix() const { return rotation.to_matrix(); }
    };

    // Transform composition: T_AC = T_AB * T_BC
    template <typename A, typename B, typename C, typename T>
    Transform<A, C, T> operator*(const Transform<A, B, T> &t_ab, const Transform<B, C, T> &t_bc) {
        return Transform<A, C, T>{t_ab.rotation * t_bc.rotation,
                                  t_ab.rotation.apply(t_bc.translation) + t_ab.translation};
    }

    // Apply transform to point: T * p
    template <typename To, typename From, typename T>
    dp::Point operator*(const Transform<To, From, T> &t, const dp::Point &p) {
        return t.apply(p);
    }

    template <typename To, typename From, typename T>
    std::ostream &operator<<(std::ostream &os, const Transform<To, From, T> &t) {
        return os << "Transform{" << t.rotation << ", t=[" << t.translation.x << "," << t.translation.y << ","
                  << t.translation.z << "]}";
    }

} // namespace concord::frame
