#include <concord/concord.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("Basic Vec3d operations") {
    SUBCASE("Vec3d creation and access") {
        Vec3d v{1.0, 2.0, 3.0};
        CHECK(v[0] == 1.0);
        CHECK(v[1] == 2.0);
        CHECK(v[2] == 3.0);
    }

    SUBCASE("Vec3d scalar multiplication") {
        Vec3d v{1.0, 2.0, 3.0};
        Vec3d result = v * 2.0;
        CHECK(result[0] == 2.0);
        CHECK(result[1] == 4.0);
        CHECK(result[2] == 6.0);
    }
}

TEST_CASE("Circle geometry tests") {
    SUBCASE("Circle creation and basic properties") {
        Point center;
        center.x = 0.0;
        center.y = 0.0;
        center.z = 0.0;

        Circle circle(center, 5.0);

        CHECK(circle.getCenter().x == 0.0);
        CHECK(circle.getCenter().y == 0.0);
        CHECK(doctest::Approx(circle.area()) == M_PI * 25.0);
        CHECK(doctest::Approx(circle.circumference()) == 2 * M_PI * 5.0);
    }

    SUBCASE("Point containment") {
        Point center;
        center.x = 0.0;
        center.y = 0.0;
        center.z = 0.0;

        Circle circle(center, 5.0);

        Point inside;
        inside.x = 2.0;
        inside.y = 2.0;
        inside.z = 0.0;

        Point outside;
        outside.x = 10.0;
        outside.y = 10.0;
        outside.z = 0.0;

        CHECK(circle.contains(inside) == true);
        CHECK(circle.contains(outside) == false);
    }
}

TEST_CASE("Point operations") {
    SUBCASE("Point creation and distance") {
        Point p1(1.0, 2.0, 3.0);
        Point p2(4.0, 6.0, 3.0);

        double dist = p1.distance_to(p2);
        CHECK(dist == doctest::Approx(5.0)); // sqrt(9 + 16) = 5
    }

    SUBCASE("Point arithmetic") {
        Point p1(1.0, 2.0, 3.0);
        Point p2(4.0, 5.0, 6.0);

        Point sum = p1 + p2;
        CHECK(sum.x == 5.0);
        CHECK(sum.y == 7.0);
        CHECK(sum.z == 9.0);
    }
}
