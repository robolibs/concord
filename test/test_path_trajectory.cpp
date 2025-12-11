#include <concord/geometry/path.hpp>
#include <concord/geometry/trajectory.hpp>
#include <concord/types/state.hpp>
#include <doctest/doctest.h>

using namespace concord;

TEST_CASE("Path with Poses") {
    SUBCASE("Default construction") {
        Path path;
        CHECK(path.empty() == true);
        CHECK(path.size() == 0);
    }

    SUBCASE("Add poses") {
        Path path;

        Pose p1{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}};
        Pose p2{Point{1.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.5}};
        Pose p3{Point{2.0, 1.0, 0.0}, Euler{0.0, 0.0, 1.0}};

        path.add(p1);
        path.add(p2);
        path.add(p3);

        CHECK(path.size() == 3);
        CHECK(path.empty() == false);
    }

    SUBCASE("Construct from vector") {
        std::vector<Pose> poses = {
            Pose{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}},
            Pose{Point{1.0, 1.0, 0.0}, Euler{0.0, 0.0, 0.5}},
            Pose{Point{2.0, 2.0, 0.0}, Euler{0.0, 0.0, 1.0}},
        };

        Path path(poses);
        CHECK(path.size() == 3);
    }

    SUBCASE("Access by index") {
        Path path;
        Pose p1{Point{1.0, 2.0, 3.0}, Euler{0.1, 0.2, 0.3}};
        Pose p2{Point{4.0, 5.0, 6.0}, Euler{0.4, 0.5, 0.6}};

        path.add(p1);
        path.add(p2);

        CHECK(path[0].point.x == doctest::Approx(1.0));
        CHECK(path[0].point.y == doctest::Approx(2.0));
        CHECK(path[1].point.x == doctest::Approx(4.0));
        CHECK(path[1].angle.yaw == doctest::Approx(0.6));
    }

    SUBCASE("Modify by index") {
        Path path;
        path.add(Pose{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}});

        path[0].point.x = 10.0;
        CHECK(path[0].point.x == doctest::Approx(10.0));
    }

    SUBCASE("Clear path") {
        Path path;
        path.add(Pose{Point{1.0, 1.0, 0.0}, Euler{0.0, 0.0, 0.0}});
        path.add(Pose{Point{2.0, 2.0, 0.0}, Euler{0.0, 0.0, 0.0}});

        CHECK(path.size() == 2);
        path.clear();
        CHECK(path.size() == 0);
        CHECK(path.empty() == true);
    }

    SUBCASE("Iterate over path") {
        Path path;
        path.add(Pose{Point{1.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}});
        path.add(Pose{Point{2.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}});
        path.add(Pose{Point{3.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}});

        double sum_x = 0.0;
        for (const auto &pose : path) {
            sum_x += pose.point.x;
        }
        CHECK(sum_x == doctest::Approx(6.0));
    }

    SUBCASE("Get underlying vector") {
        Path path;
        path.add(Pose{Point{1.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}});
        path.add(Pose{Point{2.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}});

        const auto &poses = path.get();
        CHECK(poses.size() == 2);
        CHECK(poses[0].point.x == doctest::Approx(1.0));
    }
}

TEST_CASE("State") {
    SUBCASE("Default construction") {
        State state;
        CHECK(state.lin_vel == doctest::Approx(0.0));
        CHECK(state.ang_vel == doctest::Approx(0.0));
    }

    SUBCASE("Parameterized construction") {
        Pose pose{Point{1.0, 2.0, 3.0}, Euler{0.1, 0.2, 0.3}};
        State state(pose, 5.0, 0.5);

        CHECK(state.pose.point.x == doctest::Approx(1.0));
        CHECK(state.lin_vel == doctest::Approx(5.0));
        CHECK(state.ang_vel == doctest::Approx(0.5));
    }

    SUBCASE("Equality") {
        Pose pose{Point{1.0, 2.0, 0.0}, Euler{0.0, 0.0, 0.5}};
        State s1(pose, 1.0, 0.1);
        State s2(pose, 1.0, 0.1);
        State s3(pose, 2.0, 0.1);

        CHECK(s1 == s2);
        CHECK(s1 != s3);
    }
}

TEST_CASE("Trajectory with States") {
    SUBCASE("Default construction") {
        Trajectory traj;
        CHECK(traj.empty() == true);
        CHECK(traj.size() == 0);
    }

    SUBCASE("Add states") {
        Trajectory traj;

        State s1{Pose{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 0.0, 0.0};
        State s2{Pose{Point{1.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.1}}, 1.0, 0.1};
        State s3{Pose{Point{2.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.2}}, 2.0, 0.2};

        traj.add(s1);
        traj.add(s2);
        traj.add(s3);

        CHECK(traj.size() == 3);
        CHECK(traj.empty() == false);
    }

    SUBCASE("Construct from vector") {
        std::vector<State> states = {
            State{Pose{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 0.0, 0.0},
            State{Pose{Point{1.0, 1.0, 0.0}, Euler{0.0, 0.0, 0.5}}, 1.0, 0.5},
            State{Pose{Point{2.0, 2.0, 0.0}, Euler{0.0, 0.0, 1.0}}, 2.0, 1.0},
        };

        Trajectory traj(states);
        CHECK(traj.size() == 3);
    }

    SUBCASE("Access by index") {
        Trajectory traj;
        State s1{Pose{Point{1.0, 2.0, 3.0}, Euler{0.1, 0.2, 0.3}}, 5.0, 0.5};
        State s2{Pose{Point{4.0, 5.0, 6.0}, Euler{0.4, 0.5, 0.6}}, 10.0, 1.0};

        traj.add(s1);
        traj.add(s2);

        CHECK(traj[0].pose.point.x == doctest::Approx(1.0));
        CHECK(traj[0].lin_vel == doctest::Approx(5.0));
        CHECK(traj[1].pose.point.x == doctest::Approx(4.0));
        CHECK(traj[1].ang_vel == doctest::Approx(1.0));
    }

    SUBCASE("Modify by index") {
        Trajectory traj;
        traj.add(State{Pose{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 0.0, 0.0});

        traj[0].lin_vel = 100.0;
        CHECK(traj[0].lin_vel == doctest::Approx(100.0));
    }

    SUBCASE("Clear trajectory") {
        Trajectory traj;
        traj.add(State{Pose{Point{1.0, 1.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 1.0, 0.1});
        traj.add(State{Pose{Point{2.0, 2.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 2.0, 0.2});

        CHECK(traj.size() == 2);
        traj.clear();
        CHECK(traj.size() == 0);
        CHECK(traj.empty() == true);
    }

    SUBCASE("Iterate over trajectory") {
        Trajectory traj;
        traj.add(State{Pose{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 1.0, 0.0});
        traj.add(State{Pose{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 2.0, 0.0});
        traj.add(State{Pose{Point{0.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 3.0, 0.0});

        double sum_vel = 0.0;
        for (const auto &state : traj) {
            sum_vel += state.lin_vel;
        }
        CHECK(sum_vel == doctest::Approx(6.0));
    }

    SUBCASE("Get underlying vector") {
        Trajectory traj;
        traj.add(State{Pose{Point{1.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 5.0, 0.5});
        traj.add(State{Pose{Point{2.0, 0.0, 0.0}, Euler{0.0, 0.0, 0.0}}, 10.0, 1.0});

        const auto &states = traj.get();
        CHECK(states.size() == 2);
        CHECK(states[0].lin_vel == doctest::Approx(5.0));
        CHECK(states[1].lin_vel == doctest::Approx(10.0));
    }
}
