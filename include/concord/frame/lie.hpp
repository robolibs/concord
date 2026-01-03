#pragma once

// ============================================================================
// concord/frame/lie.hpp - Lie group operations for frame-safe rotations/transforms
// ============================================================================
//
// This file provides Lie group operations (exp, log, slerp, average, Jacobians)
// for concord's frame-tagged Rotation<To,From> and Transform<To,From> types.
//
// DATA OWNERSHIP: All data is owned by datapod types (dp::Quaternion, dp::Point).
// Concord provides frame-safe wrappers, optinum provides the math.
//
// These are FREE FUNCTIONS that:
// 1. Take concord's frame-tagged types
// 2. Convert to optinum's on::lie::SO3/SE3
// 3. Call optinum's Lie group operations
// 4. Return results with proper frame tags preserved

#include "transform.hpp"

#include <optinum/lie/lie.hpp>

#include <span>

namespace concord::frame {

    namespace on = ::optinum;
    namespace dp = ::datapod;

    // ============================================================================
    // Type aliases for tangent vectors (Lie algebra elements)
    // ============================================================================

    /// Tangent vector for SO(3) - rotation vector (axis-angle)
    using RotationTangent = dp::mat::Vector<double, 3>;

    /// Tangent vector for SE(3) - twist [v, omega] where v is translational, omega is rotational
    using TransformTangent = dp::mat::Vector<double, 6>;

    // ============================================================================
    // Conversion helpers (zero-copy where possible)
    // ============================================================================

    namespace detail {

        /// Convert concord Rotation to optinum SO3
        template <typename To, typename From, typename T>
        inline on::lie::SO3<T> to_so3(const Rotation<To, From, T> &r) {
            const auto &q = r.quaternion();
            return on::lie::SO3<T>(q.w, q.x, q.y, q.z);
        }

        /// Convert optinum SO3 to concord Rotation
        template <typename To, typename From, typename T>
        inline Rotation<To, From, T> from_so3(const on::lie::SO3<T> &so3) {
            const auto &q = so3.unit_quaternion();
            return Rotation<To, From, T>{dp::Quaternion{q.w, q.x, q.y, q.z}};
        }

        /// Convert concord Transform to optinum SE3
        template <typename To, typename From, typename T>
        inline on::lie::SE3<T> to_se3(const Transform<To, From, T> &tf) {
            auto so3 = to_so3<To, From, T>(tf.rotation);
            typename on::lie::SE3<T>::Translation t{tf.translation.x, tf.translation.y, tf.translation.z};
            return on::lie::SE3<T>(so3, t);
        }

        /// Convert optinum SE3 to concord Transform
        template <typename To, typename From, typename T>
        inline Transform<To, From, T> from_se3(const on::lie::SE3<T> &se3) {
            auto rot = from_so3<To, From, T>(se3.so3());
            const auto &t = se3.translation();
            return Transform<To, From, T>{rot, dp::Point{t[0], t[1], t[2]}};
        }

    } // namespace detail

    // ============================================================================
    // Exponential and Logarithmic maps for Rotation (SO3)
    // ============================================================================

    /// Exponential map: tangent vector (rotation vector) -> Rotation
    /// The rotation vector encodes axis * angle (axis-angle representation)
    template <typename To, typename From, typename T = double>
    inline Rotation<To, From, T> exp(const RotationTangent &omega) {
        auto so3 = on::lie::SO3<T>::exp(omega);
        return detail::from_so3<To, From, T>(so3);
    }

    /// Logarithmic map: Rotation -> tangent vector (rotation vector)
    /// Returns the axis-angle representation as a 3-vector
    template <typename To, typename From, typename T> inline RotationTangent log(const Rotation<To, From, T> &r) {
        auto so3 = detail::to_so3<To, From, T>(r);
        return so3.log();
    }

    // ============================================================================
    // Exponential and Logarithmic maps for Transform (SE3)
    // ============================================================================

    /// Exponential map: twist vector -> Transform
    /// Twist = [v, omega] where v is translational velocity, omega is angular velocity
    template <typename To, typename From, typename T = double>
    inline Transform<To, From, T> exp(const TransformTangent &twist) {
        auto se3 = on::lie::SE3<T>::exp(twist);
        return detail::from_se3<To, From, T>(se3);
    }

    /// Logarithmic map: Transform -> twist vector
    template <typename To, typename From, typename T> inline TransformTangent log(const Transform<To, From, T> &tf) {
        auto se3 = detail::to_se3<To, From, T>(tf);
        return se3.log();
    }

    // ============================================================================
    // Interpolation (geodesic / SLERP)
    // ============================================================================

    /// Spherical linear interpolation between two rotations
    /// t=0 returns a, t=1 returns b, intermediate values interpolate along geodesic
    template <typename To, typename From, typename T>
    inline Rotation<To, From, T> slerp(const Rotation<To, From, T> &a, const Rotation<To, From, T> &b, T t) {
        auto so3_a = detail::to_so3<To, From, T>(a);
        auto so3_b = detail::to_so3<To, From, T>(b);
        auto result = on::lie::slerp(so3_a, so3_b, t);
        return detail::from_so3<To, From, T>(result);
    }

    /// Geodesic interpolation between two rotations (same as slerp for SO3)
    template <typename To, typename From, typename T>
    inline Rotation<To, From, T> interpolate(const Rotation<To, From, T> &a, const Rotation<To, From, T> &b, T t) {
        auto so3_a = detail::to_so3<To, From, T>(a);
        auto so3_b = detail::to_so3<To, From, T>(b);
        auto result = on::lie::interpolate(so3_a, so3_b, t);
        return detail::from_so3<To, From, T>(result);
    }

    /// Geodesic interpolation between two transforms
    /// Interpolates rotation via SLERP and translation appropriately
    template <typename To, typename From, typename T>
    inline Transform<To, From, T> interpolate(const Transform<To, From, T> &a, const Transform<To, From, T> &b, T t) {
        auto se3_a = detail::to_se3<To, From, T>(a);
        auto se3_b = detail::to_se3<To, From, T>(b);
        auto result = on::lie::interpolate(se3_a, se3_b, t);
        return detail::from_se3<To, From, T>(result);
    }

    // ============================================================================
    // Averaging (Karcher / Frechet mean)
    // ============================================================================

    /// Compute the biinvariant (Karcher/Frechet) mean of multiple rotations
    /// This is the geodesic center that minimizes sum of squared distances
    template <typename To, typename From, typename T>
    inline dp::Optional<Rotation<To, From, T>> average(std::span<const Rotation<To, From, T>> rotations,
                                                       std::size_t max_iter = 20, T tolerance = T(1e-10)) {
        if (rotations.empty()) {
            return dp::nullopt;
        }
        if (rotations.size() == 1) {
            return rotations[0];
        }

        // Convert to SO3 vector
        std::vector<on::lie::SO3<T>> so3_vec;
        so3_vec.reserve(rotations.size());
        for (const auto &r : rotations) {
            so3_vec.push_back(detail::to_so3<To, From, T>(r));
        }

        // Use optinum's average
        auto result = on::lie::average<on::lie::SO3<T>>(so3_vec, max_iter, tolerance);
        if (result) {
            return detail::from_so3<To, From, T>(*result);
        }
        return dp::nullopt;
    }

    /// Compute the biinvariant mean of multiple transforms
    template <typename To, typename From, typename T>
    inline dp::Optional<Transform<To, From, T>> average(std::span<const Transform<To, From, T>> transforms,
                                                        std::size_t max_iter = 20, T tolerance = T(1e-10)) {
        if (transforms.empty()) {
            return dp::nullopt;
        }
        if (transforms.size() == 1) {
            return transforms[0];
        }

        // Convert to SE3 vector
        std::vector<on::lie::SE3<T>> se3_vec;
        se3_vec.reserve(transforms.size());
        for (const auto &tf : transforms) {
            se3_vec.push_back(detail::to_se3<To, From, T>(tf));
        }

        // Use optinum's average
        auto result = on::lie::average<on::lie::SE3<T>>(se3_vec, max_iter, tolerance);
        if (result) {
            return detail::from_se3<To, From, T>(*result);
        }
        return dp::nullopt;
    }

    /// Fast average of exactly two rotations (midpoint on geodesic)
    template <typename To, typename From, typename T>
    inline Rotation<To, From, T> average_two(const Rotation<To, From, T> &a, const Rotation<To, From, T> &b) {
        auto so3_a = detail::to_so3<To, From, T>(a);
        auto so3_b = detail::to_so3<To, From, T>(b);
        auto result = on::lie::average_two(so3_a, so3_b);
        return detail::from_so3<To, From, T>(result);
    }

    /// Fast average of exactly two transforms
    template <typename To, typename From, typename T>
    inline Transform<To, From, T> average_two(const Transform<To, From, T> &a, const Transform<To, From, T> &b) {
        auto se3_a = detail::to_se3<To, From, T>(a);
        auto se3_b = detail::to_se3<To, From, T>(b);
        auto result = on::lie::average_two(se3_a, se3_b);
        return detail::from_se3<To, From, T>(result);
    }

    // ============================================================================
    // Jacobians (for optimization)
    // ============================================================================

    /// Left Jacobian of SO3 at tangent vector omega
    /// J_l maps perturbations in tangent space to perturbations in group
    template <typename T = double>
    inline typename on::lie::SO3<T>::AdjointMatrix left_jacobian_so3(const RotationTangent &omega) {
        return on::lie::SO3<T>::left_jacobian(omega);
    }

    /// Inverse of left Jacobian of SO3
    template <typename T = double>
    inline typename on::lie::SO3<T>::AdjointMatrix left_jacobian_inverse_so3(const RotationTangent &omega) {
        return on::lie::SO3<T>::left_jacobian_inverse(omega);
    }

    /// Left Jacobian of SE3 at twist vector
    template <typename T = double>
    inline typename on::lie::SE3<T>::AdjointMatrix left_jacobian_se3(const TransformTangent &twist) {
        return on::lie::SE3<T>::left_jacobian(twist);
    }

    /// Inverse of left Jacobian of SE3
    template <typename T = double>
    inline typename on::lie::SE3<T>::AdjointMatrix left_jacobian_inverse_se3(const TransformTangent &twist) {
        return on::lie::SE3<T>::left_jacobian_inverse(twist);
    }

    // ============================================================================
    // Adjoint representation
    // ============================================================================

    /// Adjoint matrix of a rotation (for transforming tangent vectors)
    /// For SO3, Adj(R) = R (the rotation matrix itself)
    template <typename To, typename From, typename T>
    inline typename on::lie::SO3<T>::AdjointMatrix adjoint(const Rotation<To, From, T> &r) {
        auto so3 = detail::to_so3<To, From, T>(r);
        return so3.Adj();
    }

    /// Adjoint matrix of a transform
    /// For SE3, Adj(T) is a 6x6 matrix
    template <typename To, typename From, typename T>
    inline typename on::lie::SE3<T>::AdjointMatrix adjoint(const Transform<To, From, T> &tf) {
        auto se3 = detail::to_se3<To, From, T>(tf);
        return se3.Adj();
    }

    // ============================================================================
    // Hat and Vee operators (Lie algebra <-> matrix)
    // ============================================================================

    /// Hat operator: rotation vector -> 3x3 skew-symmetric matrix
    template <typename T = double>
    inline typename on::lie::SO3<T>::RotationMatrix hat_so3(const RotationTangent &omega) {
        return on::lie::SO3<T>::hat(omega);
    }

    /// Vee operator: 3x3 skew-symmetric matrix -> rotation vector
    template <typename T = double>
    inline RotationTangent vee_so3(const typename on::lie::SO3<T>::RotationMatrix &Omega) {
        return on::lie::SO3<T>::vee(Omega);
    }

    /// Hat operator: twist -> 4x4 se(3) matrix
    template <typename T = double>
    inline typename on::lie::SE3<T>::HomogeneousMatrix hat_se3(const TransformTangent &twist) {
        return on::lie::SE3<T>::hat(twist);
    }

    /// Vee operator: 4x4 se(3) matrix -> twist
    template <typename T = double>
    inline TransformTangent vee_se3(const typename on::lie::SE3<T>::HomogeneousMatrix &Omega) {
        return on::lie::SE3<T>::vee(Omega);
    }

    // ============================================================================
    // Utility functions
    // ============================================================================

    /// Compute rotation angle (magnitude of rotation)
    template <typename To, typename From, typename T> inline T angle(const Rotation<To, From, T> &r) {
        auto so3 = detail::to_so3<To, From, T>(r);
        return so3.angle();
    }

    /// Compute rotation axis (unit vector, undefined for identity)
    template <typename To, typename From, typename T> inline dp::Point axis(const Rotation<To, From, T> &r) {
        auto so3 = detail::to_so3<To, From, T>(r);
        auto ax = so3.axis();
        return dp::Point{ax[0], ax[1], ax[2]};
    }

    /// Check if rotation is approximately identity
    template <typename To, typename From, typename T>
    inline bool is_identity(const Rotation<To, From, T> &r, T tolerance = T(1e-10)) {
        auto so3 = detail::to_so3<To, From, T>(r);
        return so3.is_identity(tolerance);
    }

    /// Check if transform is approximately identity
    template <typename To, typename From, typename T>
    inline bool is_identity(const Transform<To, From, T> &tf, T tolerance = T(1e-10)) {
        auto se3 = detail::to_se3<To, From, T>(tf);
        return se3.is_identity(tolerance);
    }

    /// Check if two rotations are approximately equal
    template <typename To, typename From, typename T>
    inline bool is_approx(const Rotation<To, From, T> &a, const Rotation<To, From, T> &b, T tolerance = T(1e-10)) {
        auto so3_a = detail::to_so3<To, From, T>(a);
        auto so3_b = detail::to_so3<To, From, T>(b);
        return so3_a.is_approx(so3_b, tolerance);
    }

    /// Check if two transforms are approximately equal
    template <typename To, typename From, typename T>
    inline bool is_approx(const Transform<To, From, T> &a, const Transform<To, From, T> &b, T tolerance = T(1e-10)) {
        auto se3_a = detail::to_se3<To, From, T>(a);
        auto se3_b = detail::to_se3<To, From, T>(b);
        return se3_a.is_approx(se3_b, tolerance);
    }

} // namespace concord::frame
