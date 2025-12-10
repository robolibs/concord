#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace concord {

    // Basic 3D vector type
    using Vec3d = std::array<double, 3>;

    // Vec3d operations
    inline Vec3d operator+(const Vec3d &a, const Vec3d &b) { return Vec3d{a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
    inline Vec3d operator-(const Vec3d &a, const Vec3d &b) { return Vec3d{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
    inline Vec3d operator*(const Vec3d &v, double s) { return Vec3d{v[0] * s, v[1] * s, v[2] * s}; }
    inline Vec3d operator*(double s, const Vec3d &v) { return Vec3d{v[0] * s, v[1] * s, v[2] * s}; }

    // Basic 3x3 matrix type
    struct Mat3d {
        std::array<std::array<double, 3>, 3> data{};

        inline std::array<double, 3> &operator[](size_t i) { return data[i]; }
        inline const std::array<double, 3> &operator[](size_t i) const { return data[i]; }

        inline Vec3d operator*(const Vec3d &v) const {
            return Vec3d{data[0][0] * v[0] + data[0][1] * v[1] + data[0][2] * v[2],
                         data[1][0] * v[0] + data[1][1] * v[1] + data[1][2] * v[2],
                         data[2][0] * v[0] + data[2][1] * v[1] + data[2][2] * v[2]};
        }
    };

    // Mathematical exception for error handling
    class MathematicalException : public std::runtime_error {
      public:
        explicit MathematicalException(const std::string &msg) : std::runtime_error(msg) {}
    };

    // Validation utilities
    namespace validation {
        inline void validate_finite(double value, const char *name) {
            if (!std::isfinite(value)) {
                throw MathematicalException(std::string(name) + " must be finite");
            }
        }

        inline void validate_latitude(double lat) {
            if (lat < -90.0 || lat > 90.0) {
                throw MathematicalException("latitude must be in range [-90, 90]");
            }
        }

        inline void validate_longitude(double lon) {
            if (lon < -180.0 || lon > 180.0) {
                throw MathematicalException("longitude must be in range [-180, 180]");
            }
        }

        inline void validate_altitude(double alt) {
            if (alt < -12000.0 || alt > 100000.0) {
                throw MathematicalException("altitude must be in reasonable range [-12000, 100000]");
            }
        }
    } // namespace validation

    // Safe math utilities
    namespace safe_math {
        inline double safe_sqrt(double value, const char * /* context */ = "") {
            if (value < 0.0) {
                return 0.0;
            }
            return std::sqrt(value);
        }

        inline double safe_asin(double value, const char * /* context */ = "") {
            if (value < -1.0)
                value = -1.0;
            if (value > 1.0)
                value = 1.0;
            return std::asin(value);
        }
    } // namespace safe_math

} // namespace concord
