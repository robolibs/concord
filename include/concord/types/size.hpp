#pragma once

#include <algorithm>
#include <cmath>

namespace concord {

    struct Size {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        Size() = default;
        Size(double x_, double y_, double z_);
        Size(double s); // Uniform scaling
        bool is_set() const;

        // Mathematical operations
        Size operator+(const Size &other) const;
        Size operator-(const Size &other) const;
        Size operator*(double scale) const;
        Size operator/(double scale) const;
        Size operator*(const Size &other) const;

        bool operator==(const Size &other) const;
        bool operator!=(const Size &other) const;

        // Volume and area calculations
        double volume() const;
        double area_xy() const;
        double area_xz() const;
        double area_yz() const;
        double diagonal() const;
        double diagonal_2d() const;

        // Utility functions
        Size abs() const;
        Size max(const Size &other) const;
        Size min(const Size &other) const;
    };

} // namespace concord
