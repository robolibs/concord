#pragma once

#include <array>
#include <cmath>
#include <initializer_list>
#include <algorithm>
#include <numeric>

namespace concord {

    // Vector 2D/3D types for general math operations
    template<typename T, size_t N>
    struct Vector {
        std::array<T, N> data{};
        
        Vector() = default;
        Vector(std::initializer_list<T> init) {
            auto it = init.begin();
            for (size_t i = 0; i < N && it != init.end(); ++i, ++it) {
                data[i] = *it;
            }
        }
        
        inline T& operator[](size_t i) { return data[i]; }
        inline const T& operator[](size_t i) const { return data[i]; }
        
        inline Vector operator+(const Vector& other) const {
            Vector result;
            for (size_t i = 0; i < N; ++i) {
                result[i] = data[i] + other[i];
            }
            return result;
        }
        
        inline Vector operator-(const Vector& other) const {
            Vector result;
            for (size_t i = 0; i < N; ++i) {
                result[i] = data[i] - other[i];
            }
            return result;
        }
        
        inline Vector operator*(T scalar) const {
            Vector result;
            for (size_t i = 0; i < N; ++i) {
                result[i] = data[i] * scalar;
            }
            return result;
        }
        
        inline T dot(const Vector& other) const {
            T result = T{};
            for (size_t i = 0; i < N; ++i) {
                result += data[i] * other[i];
            }
            return result;
        }
        
        inline T magnitude() const {
            return std::sqrt(dot(*this));
        }
        
        inline Vector normalized() const {
            T mag = magnitude();
            if (mag > 1e-10) {
                return *this * (T(1) / mag);
            }
            return *this;
        }
        
        inline bool operator==(const Vector& other) const {
            for (size_t i = 0; i < N; ++i) {
                if (data[i] != other[i]) return false;
            }
            return true;
        }
        
        inline bool operator!=(const Vector& other) const {
            return !(*this == other);
        }
    };
    
    using Vec2f = Vector<float, 2>;
    using Vec3f = Vector<float, 3>;
    using Vec2d = Vector<double, 2>;
    using Vec3d = Vector<double, 3>;
    
    // Cross product for 3D vectors
    template<typename T>
    inline Vector<T, 3> cross(const Vector<T, 3>& a, const Vector<T, 3>& b) {
        return Vector<T, 3>{
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }
    
    // Matrix 3x3 and 4x4 types for transformations
    template<typename T, size_t Rows, size_t Cols>
    struct Matrix {
        std::array<std::array<T, Cols>, Rows> data{};
        
        Matrix() {
            // Initialize to identity if square matrix
            if constexpr (Rows == Cols) {
                for (size_t i = 0; i < Rows; ++i) {
                    data[i][i] = T(1);
                }
            }
        }
        
        inline std::array<T, Cols>& operator[](size_t row) { return data[row]; }
        inline const std::array<T, Cols>& operator[](size_t row) const { return data[row]; }
        
        template<size_t OtherCols>
        inline Matrix<T, Rows, OtherCols> operator*(const Matrix<T, Cols, OtherCols>& other) const {
            Matrix<T, Rows, OtherCols> result{};
            for (size_t i = 0; i < Rows; ++i) {
                for (size_t j = 0; j < OtherCols; ++j) {
                    result[i][j] = T{};
                    for (size_t k = 0; k < Cols; ++k) {
                        result[i][j] += data[i][k] * other[k][j];
                    }
                }
            }
            return result;
        }
        
        inline Vector<T, Rows> operator*(const Vector<T, Cols>& vec) const {
            Vector<T, Rows> result;
            for (size_t i = 0; i < Rows; ++i) {
                result[i] = T{};
                for (size_t j = 0; j < Cols; ++j) {
                    result[i] += data[i][j] * vec[j];
                }
            }
            return result;
        }
        
        inline static Matrix<T, Rows, Cols> identity() {
            static_assert(Rows == Cols, "Identity matrix must be square");
            return Matrix<T, Rows, Cols>{};
        }
        
        inline bool operator==(const Matrix& other) const {
            for (size_t i = 0; i < Rows; ++i) {
                for (size_t j = 0; j < Cols; ++j) {
                    if (data[i][j] != other[i][j]) return false;
                }
            }
            return true;
        }
        
        inline bool operator!=(const Matrix& other) const {
            return !(*this == other);
        }
    };
    
    using Mat3f = Matrix<float, 3, 3>;
    using Mat4f = Matrix<float, 4, 4>;
    using Mat3d = Matrix<double, 3, 3>;
    using Mat4d = Matrix<double, 4, 4>;
    
    // Transformation matrices
    template<typename T>
    inline Matrix<T, 3, 3> rotationMatrix2D(T angle) {
        T c = std::cos(angle);
        T s = std::sin(angle);
        Matrix<T, 3, 3> result{};
        result[0][0] = c;  result[0][1] = -s; result[0][2] = T(0);
        result[1][0] = s;  result[1][1] = c;  result[1][2] = T(0);
        result[2][0] = T(0); result[2][1] = T(0); result[2][2] = T(1);
        return result;
    }
    
    template<typename T>
    inline Matrix<T, 4, 4> translationMatrix(T x, T y, T z) {
        Matrix<T, 4, 4> result{};
        result[0][3] = x;
        result[1][3] = y;
        result[2][3] = z;
        return result;
    }
    
    // Interpolation utilities
    template<typename T>
    inline T lerp(T a, T b, T t) {
        return a + t * (b - a);
    }
    
    template<typename T>
    inline T smoothstep(T edge0, T edge1, T x) {
        T t = std::clamp((x - edge0) / (edge1 - edge0), T(0), T(1));
        return t * t * (T(3) - T(2) * t);
    }
    
    // Global mathematical functions
    
    // Dot product for Vec3d
    inline double dot_product(const Vec3d& a, const Vec3d& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
    
    // Cross product for Vec3d
    inline Vec3d cross_product(const Vec3d& a, const Vec3d& b) {
        Vec3d result;
        result[0] = a[1] * b[2] - a[2] * b[1];
        result[1] = a[2] * b[0] - a[0] * b[2];
        result[2] = a[0] * b[1] - a[1] * b[0];
        return result;
    }
    
    // Create rotation matrix around Z axis
    inline Mat3d create_rotation_z(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        Mat3d result;
        result[0][0] = c;  result[0][1] = -s; result[0][2] = 0;
        result[1][0] = s;  result[1][1] = c;  result[1][2] = 0;
        result[2][0] = 0;  result[2][1] = 0;  result[2][2] = 1;
        return result;
    }
    
    // Create rotation matrix around X axis
    inline Mat3d create_rotation_x(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        Mat3d result;
        result[0][0] = 1;  result[0][1] = 0;  result[0][2] = 0;
        result[1][0] = 0;  result[1][1] = c;  result[1][2] = -s;
        result[2][0] = 0;  result[2][1] = s;  result[2][2] = c;
        return result;
    }
    
    // Create rotation matrix around Y axis
    inline Mat3d create_rotation_y(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        Mat3d result;
        result[0][0] = c;   result[0][1] = 0;  result[0][2] = s;
        result[1][0] = 0;   result[1][1] = 1;  result[1][2] = 0;
        result[2][0] = -s;  result[2][1] = 0;  result[2][2] = c;
        return result;
    }
    
    // Linear interpolation
    inline double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }
    
    // Smooth interpolation using smoothstep
    inline double smoothstep(double edge0, double edge1, double x) {
        double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }
    
    // Constants
    template<typename T>
    constexpr T PI = T(3.14159265358979323846);
    
    template<typename T>
    constexpr T DEG_TO_RAD = concord::PI<T> / T(180);
    
    template<typename T>
    constexpr T RAD_TO_DEG = T(180) / concord::PI<T>;

} // namespace concord
