#pragma once

#include <cmath>
#include <algorithm>

namespace concord {

    struct Size {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        Size() = default;
        Size(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
        Size(double s) : x(s), y(s), z(s) {} // Uniform scaling
        inline bool is_set() const { return x != 0.0 || y != 0.0 || z != 0.0; }
        
        // Mathematical operations
        inline Size operator+(const Size& other) const { return Size{x + other.x, y + other.y, z + other.z}; }
        inline Size operator-(const Size& other) const { return Size{x - other.x, y - other.y, z - other.z}; }
        inline Size operator*(double scale) const { return Size{x * scale, y * scale, z * scale}; }
        inline Size operator/(double scale) const { return Size{x / scale, y / scale, z / scale}; }
        inline Size operator*(const Size& other) const { return Size{x * other.x, y * other.y, z * other.z}; }
        
        inline bool operator==(const Size& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        inline bool operator!=(const Size& other) const {
            return !(*this == other);
        }
        
        // Volume and area calculations
        inline double volume() const { return x * y * z; }
        inline double area_xy() const { return x * y; }
        inline double area_xz() const { return x * z; }
        inline double area_yz() const { return y * z; }
        inline double diagonal() const { return std::sqrt(x*x + y*y + z*z); }
        inline double diagonal_2d() const { return std::sqrt(x*x + y*y); }
        
        // Utility functions
        inline Size abs() const { return Size{std::abs(x), std::abs(y), std::abs(z)}; }
        inline Size max(const Size& other) const { 
            return Size{std::max(x, other.x), std::max(y, other.y), std::max(z, other.z)}; 
        }
        inline Size min(const Size& other) const { 
            return Size{std::min(x, other.x), std::min(y, other.y), std::min(z, other.z)}; 
        }
    };

} // namespace concord
