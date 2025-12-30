#pragma once

// ============================================================================
// concord/frame/spline.hpp - Smooth spline interpolation for frame-safe types
// ============================================================================
//
// Provides C1-smooth spline interpolation through sequences of rotations and
// transforms, maintaining frame safety through template parameters.
//
// DATA OWNERSHIP: Spline control points are stored in std::vector (could use
// dp::Vector). The underlying Lie group math is delegated to optinum.
//
// Usage:
//   // Create a rotation spline
//   RotationSpline<World, Body> spline;
//   spline.add_point(rot1);
//   spline.add_point(rot2);
//   spline.add_point(rot3);
//   spline.build();
//
//   // Evaluate at parameter t in [0, 1]
//   auto rot = spline.evaluate(0.5);
//
//   // Or with timestamps
//   TimedRotationSpline<World, Body> timed_spline;
//   timed_spline.add_point(t1, rot1);
//   timed_spline.add_point(t2, rot2);
//   auto rot = timed_spline.evaluate_at(query_time);

#include "lie.hpp"
#include "transform.hpp"

#include <optinum/lie/algorithms/spline.hpp>

#include <vector>

namespace concord::frame {

    namespace on = ::optinum;

    // ============================================================================
    // RotationSpline - C1-smooth spline through rotations
    // ============================================================================

    /// Smooth spline interpolation through a sequence of rotations
    /// Maintains frame safety: all rotations are To<-From
    template <typename ToFrame, typename FromFrame, typename T = double> class RotationSpline {
      public:
        using RotationType = Rotation<ToFrame, FromFrame, T>;
        using TangentType = RotationTangent;

      private:
        std::vector<RotationType> points_;
        on::lie::LieSpline<on::lie::SO3<T>> spline_;
        bool built_ = false;

      public:
        RotationSpline() = default;

        /// Construct from a vector of rotations
        explicit RotationSpline(const std::vector<RotationType> &points) : points_(points) {
            if (points_.size() >= 2) {
                build();
            }
        }

        /// Add a control point
        void add_point(const RotationType &rot) {
            points_.push_back(rot);
            built_ = false;
        }

        /// Clear all control points
        void clear() {
            points_.clear();
            built_ = false;
        }

        /// Number of control points
        [[nodiscard]] std::size_t size() const noexcept { return points_.size(); }

        /// Check if spline is ready for evaluation
        [[nodiscard]] bool is_built() const noexcept { return built_; }

        /// Build the spline (must be called after adding points)
        void build() {
            if (points_.size() < 2) {
                built_ = false;
                return;
            }

            // Convert to optinum SO3 vector
            std::vector<on::lie::SO3<T>> so3_points;
            so3_points.reserve(points_.size());
            for (const auto &rot : points_) {
                so3_points.push_back(detail::to_so3<ToFrame, FromFrame, T>(rot));
            }

            spline_ = on::lie::LieSpline<on::lie::SO3<T>>(so3_points);
            built_ = true;
        }

        /// Evaluate spline at parameter t in [0, n-1] where n is number of points
        [[nodiscard]] RotationType evaluate(T t) const {
            if (!built_ || points_.empty()) {
                return RotationType::identity();
            }
            auto so3 = spline_.evaluate(t);
            return detail::from_so3<ToFrame, FromFrame, T>(so3);
        }

        /// Evaluate spline at normalized parameter u in [0, 1]
        [[nodiscard]] RotationType evaluate_normalized(T u) const {
            if (!built_ || points_.empty()) {
                return RotationType::identity();
            }
            auto so3 = spline_.evaluate_normalized(u);
            return detail::from_so3<ToFrame, FromFrame, T>(so3);
        }

        /// Get control point at index
        [[nodiscard]] const RotationType &point(std::size_t i) const { return points_[i]; }
    };

    // ============================================================================
    // TransformSpline - C1-smooth spline through transforms
    // ============================================================================

    /// Smooth spline interpolation through a sequence of transforms
    /// Maintains frame safety: all transforms are To<-From
    template <typename ToFrame, typename FromFrame, typename T = double> class TransformSpline {
      public:
        using TransformType = Transform<ToFrame, FromFrame, T>;
        using TangentType = TransformTangent;

      private:
        std::vector<TransformType> points_;
        on::lie::LieSpline<on::lie::SE3<T>> spline_;
        bool built_ = false;

      public:
        TransformSpline() = default;

        /// Construct from a vector of transforms
        explicit TransformSpline(const std::vector<TransformType> &points) : points_(points) {
            if (points_.size() >= 2) {
                build();
            }
        }

        /// Add a control point
        void add_point(const TransformType &tf) {
            points_.push_back(tf);
            built_ = false;
        }

        /// Clear all control points
        void clear() {
            points_.clear();
            built_ = false;
        }

        /// Number of control points
        [[nodiscard]] std::size_t size() const noexcept { return points_.size(); }

        /// Check if spline is ready for evaluation
        [[nodiscard]] bool is_built() const noexcept { return built_; }

        /// Build the spline (must be called after adding points)
        void build() {
            if (points_.size() < 2) {
                built_ = false;
                return;
            }

            // Convert to optinum SE3 vector
            std::vector<on::lie::SE3<T>> se3_points;
            se3_points.reserve(points_.size());
            for (const auto &tf : points_) {
                se3_points.push_back(detail::to_se3<ToFrame, FromFrame, T>(tf));
            }

            spline_ = on::lie::LieSpline<on::lie::SE3<T>>(se3_points);
            built_ = true;
        }

        /// Evaluate spline at parameter t in [0, n-1] where n is number of points
        [[nodiscard]] TransformType evaluate(T t) const {
            if (!built_ || points_.empty()) {
                return TransformType::identity();
            }
            auto se3 = spline_.evaluate(t);
            return detail::from_se3<ToFrame, FromFrame, T>(se3);
        }

        /// Evaluate spline at normalized parameter u in [0, 1]
        [[nodiscard]] TransformType evaluate_normalized(T u) const {
            if (!built_ || points_.empty()) {
                return TransformType::identity();
            }
            auto se3 = spline_.evaluate_normalized(u);
            return detail::from_se3<ToFrame, FromFrame, T>(se3);
        }

        /// Get control point at index
        [[nodiscard]] const TransformType &point(std::size_t i) const { return points_[i]; }
    };

    // ============================================================================
    // TimedRotationSpline - Rotation spline with timestamps
    // ============================================================================

    /// Rotation spline with timestamp-based evaluation
    template <typename ToFrame, typename FromFrame, typename T = double> class TimedRotationSpline {
      public:
        using RotationType = Rotation<ToFrame, FromFrame, T>;

      private:
        std::vector<T> times_;
        std::vector<RotationType> points_;
        on::lie::LieSpline<on::lie::SO3<T>> spline_;
        bool built_ = false;

      public:
        TimedRotationSpline() = default;

        /// Add a timestamped control point
        void add_point(T time, const RotationType &rot) {
            // Insert in sorted order
            auto it = std::lower_bound(times_.begin(), times_.end(), time);
            auto idx = std::distance(times_.begin(), it);
            times_.insert(it, time);
            points_.insert(points_.begin() + idx, rot);
            built_ = false;
        }

        /// Clear all control points
        void clear() {
            times_.clear();
            points_.clear();
            built_ = false;
        }

        /// Number of control points
        [[nodiscard]] std::size_t size() const noexcept { return points_.size(); }

        /// Check if spline is ready for evaluation
        [[nodiscard]] bool is_built() const noexcept { return built_; }

        /// Get time range [min, max]
        [[nodiscard]] std::pair<T, T> time_range() const {
            if (times_.empty()) {
                return {T(0), T(0)};
            }
            return {times_.front(), times_.back()};
        }

        /// Build the spline
        void build() {
            if (points_.size() < 2) {
                built_ = false;
                return;
            }

            std::vector<on::lie::SO3<T>> so3_points;
            so3_points.reserve(points_.size());
            for (const auto &rot : points_) {
                so3_points.push_back(detail::to_so3<ToFrame, FromFrame, T>(rot));
            }

            spline_ = on::lie::LieSpline<on::lie::SO3<T>>(so3_points);
            built_ = true;
        }

        /// Evaluate at a specific time
        [[nodiscard]] RotationType evaluate_at(T time) const {
            if (!built_ || points_.size() < 2) {
                return points_.empty() ? RotationType::identity() : points_[0];
            }

            // Clamp to time range
            time = std::max(times_.front(), std::min(time, times_.back()));

            // Find segment
            auto it = std::lower_bound(times_.begin(), times_.end(), time);
            std::size_t idx = std::distance(times_.begin(), it);
            if (idx == 0) {
                idx = 1;
            }
            if (idx >= times_.size()) {
                idx = times_.size() - 1;
            }

            // Compute local parameter
            T t0 = times_[idx - 1];
            T t1 = times_[idx];
            T local_t = (time - t0) / (t1 - t0);

            // Map to spline parameter space
            T spline_t = static_cast<T>(idx - 1) + local_t;

            auto so3 = spline_.evaluate(spline_t);
            return detail::from_so3<ToFrame, FromFrame, T>(so3);
        }

        /// Check if a time is within the valid range
        [[nodiscard]] bool can_evaluate(T time) const {
            if (times_.size() < 2) {
                return false;
            }
            return time >= times_.front() && time <= times_.back();
        }
    };

    // ============================================================================
    // TimedTransformSpline - Transform spline with timestamps
    // ============================================================================

    /// Transform spline with timestamp-based evaluation
    template <typename ToFrame, typename FromFrame, typename T = double> class TimedTransformSpline {
      public:
        using TransformType = Transform<ToFrame, FromFrame, T>;

      private:
        std::vector<T> times_;
        std::vector<TransformType> points_;
        on::lie::LieSpline<on::lie::SE3<T>> spline_;
        bool built_ = false;

      public:
        TimedTransformSpline() = default;

        /// Add a timestamped control point
        void add_point(T time, const TransformType &tf) {
            // Insert in sorted order
            auto it = std::lower_bound(times_.begin(), times_.end(), time);
            auto idx = std::distance(times_.begin(), it);
            times_.insert(it, time);
            points_.insert(points_.begin() + idx, tf);
            built_ = false;
        }

        /// Clear all control points
        void clear() {
            times_.clear();
            points_.clear();
            built_ = false;
        }

        /// Number of control points
        [[nodiscard]] std::size_t size() const noexcept { return points_.size(); }

        /// Check if spline is ready for evaluation
        [[nodiscard]] bool is_built() const noexcept { return built_; }

        /// Get time range [min, max]
        [[nodiscard]] std::pair<T, T> time_range() const {
            if (times_.empty()) {
                return {T(0), T(0)};
            }
            return {times_.front(), times_.back()};
        }

        /// Build the spline
        void build() {
            if (points_.size() < 2) {
                built_ = false;
                return;
            }

            std::vector<on::lie::SE3<T>> se3_points;
            se3_points.reserve(points_.size());
            for (const auto &tf : points_) {
                se3_points.push_back(detail::to_se3<ToFrame, FromFrame, T>(tf));
            }

            spline_ = on::lie::LieSpline<on::lie::SE3<T>>(se3_points);
            built_ = true;
        }

        /// Evaluate at a specific time
        [[nodiscard]] TransformType evaluate_at(T time) const {
            if (!built_ || points_.size() < 2) {
                return points_.empty() ? TransformType::identity() : points_[0];
            }

            // Clamp to time range
            time = std::max(times_.front(), std::min(time, times_.back()));

            // Find segment
            auto it = std::lower_bound(times_.begin(), times_.end(), time);
            std::size_t idx = std::distance(times_.begin(), it);
            if (idx == 0) {
                idx = 1;
            }
            if (idx >= times_.size()) {
                idx = times_.size() - 1;
            }

            // Compute local parameter
            T t0 = times_[idx - 1];
            T t1 = times_[idx];
            T local_t = (time - t0) / (t1 - t0);

            // Map to spline parameter space
            T spline_t = static_cast<T>(idx - 1) + local_t;

            auto se3 = spline_.evaluate(spline_t);
            return detail::from_se3<ToFrame, FromFrame, T>(se3);
        }

        /// Check if a time is within the valid range
        [[nodiscard]] bool can_evaluate(T time) const {
            if (times_.size() < 2) {
                return false;
            }
            return time >= times_.front() && time <= times_.back();
        }
    };

    // ============================================================================
    // Free functions for one-shot spline operations
    // ============================================================================

    /// Sample a rotation spline at uniform intervals
    template <typename To, typename From, typename T>
    inline std::vector<Rotation<To, From, T>> sample_rotation_spline(const RotationSpline<To, From, T> &spline,
                                                                     std::size_t num_samples) {
        std::vector<Rotation<To, From, T>> samples;
        samples.reserve(num_samples);

        if (num_samples == 0 || !spline.is_built()) {
            return samples;
        }

        const T step = T(1) / static_cast<T>(num_samples - 1);
        for (std::size_t i = 0; i < num_samples; ++i) {
            T u = static_cast<T>(i) * step;
            samples.push_back(spline.evaluate_normalized(u));
        }

        return samples;
    }

    /// Sample a transform spline at uniform intervals
    template <typename To, typename From, typename T>
    inline std::vector<Transform<To, From, T>> sample_transform_spline(const TransformSpline<To, From, T> &spline,
                                                                       std::size_t num_samples) {
        std::vector<Transform<To, From, T>> samples;
        samples.reserve(num_samples);

        if (num_samples == 0 || !spline.is_built()) {
            return samples;
        }

        const T step = T(1) / static_cast<T>(num_samples - 1);
        for (std::size_t i = 0; i < num_samples; ++i) {
            T u = static_cast<T>(i) * step;
            samples.push_back(spline.evaluate_normalized(u));
        }

        return samples;
    }

} // namespace concord::frame
