#pragma once

#include "../../types/point.hpp"
#include <cmath>
#include <vector>

namespace concord {

    /**
     * @brief A 2D triangle defined by three vertices
     *
     * Useful for line-of-sight detection, collision detection, etc.
     */
    class Triangle {
      public:
        Triangle() = default;

        /**
         * @brief Construct a triangle from three points
         * @param a First vertex
         * @param b Second vertex
         * @param c Third vertex
         */
        Triangle(const Point &a, const Point &b, const Point &c);

        /**
         * @brief Create a "vision triangle" for line-of-sight detection
         * @param apex Origin point (e.g., robot position)
         * @param heading Direction to look (radians, 0 = +X axis)
         * @param range How far to look
         * @param half_angle Half of the field of view (radians)
         * @return Triangle representing the vision cone
         *
         * Example: half_angle = PI/4 (45 deg) creates a 90 degree FOV triangle
         */
        static Triangle from_vision(const Point &apex, double heading, double range, double half_angle);

        /**
         * @brief Check if a point is inside the triangle
         * @param p Point to check
         * @return true if the point is inside the triangle
         */
        bool contains(const Point &p) const noexcept;

        /**
         * @brief Check if a circle intersects the triangle
         * @param center Center of the circle
         * @param radius Radius of the circle
         * @return true if any part of the circle is inside or touching the triangle
         */
        bool intersects_circle(const Point &center, double radius) const noexcept;

        /**
         * @brief Get the area of the triangle
         * @return Area in square units (always positive)
         */
        double area() const noexcept;

        /**
         * @brief Get the perimeter of the triangle
         * @return Perimeter length
         */
        double perimeter() const noexcept;

        /**
         * @brief Get triangle vertices as a vector
         * @return Vector of 3 points
         */
        std::vector<Point> get_vertices() const;

        // Vertex accessors
        const Point &getA() const noexcept;
        const Point &getB() const noexcept;
        const Point &getC() const noexcept;

        void setA(const Point &p) noexcept;
        void setB(const Point &p) noexcept;
        void setC(const Point &p) noexcept;

        bool operator==(const Triangle &other) const;
        bool operator!=(const Triangle &other) const;

      private:
        Point a, b, c;

        /**
         * @brief Calculate signed area (used for point-in-triangle test)
         */
        static double sign(const Point &p1, const Point &p2, const Point &p3) noexcept;
    };

} // namespace concord
