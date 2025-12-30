#include <concord/transform/listener.hpp>
#include <doctest/doctest.h>

// Define test frame tags
struct World {};
struct Odom {};
struct BaseLink {};
struct Camera {};
struct Lidar {};

// ============================================================================
// Helper functions
// ============================================================================

template <typename To, typename From>
concord::frame::Transform<To, From> make_translation(double x, double y, double z) {
    return concord::frame::Transform<To, From>{concord::frame::Rotation<To, From>::identity(), dp::Point{x, y, z}};
}

// ============================================================================
// ListenableTransformTree Tests
// ============================================================================

TEST_CASE("ListenableTransformTree - single callback registration and trigger") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count = 0;
    dp::Point last_translation{0, 0, 0};

    auto id = tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &tf) {
        call_count++;
        last_translation = tf.translation;
    });

    CHECK(id > 0);
    CHECK(tree.edge_listener_count() == 1);

    // Set transform should trigger callback
    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 2.0, 3.0));

    CHECK(call_count == 1);
    CHECK(last_translation.x == doctest::Approx(1.0));
    CHECK(last_translation.y == doctest::Approx(2.0));
    CHECK(last_translation.z == doctest::Approx(3.0));
}

TEST_CASE("ListenableTransformTree - multiple callbacks for same edge") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count_1 = 0;
    int call_count_2 = 0;

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { call_count_1++; });

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { call_count_2++; });

    CHECK(tree.edge_listener_count() == 2);

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));

    CHECK(call_count_1 == 1);
    CHECK(call_count_2 == 1);

    // Second update
    tree.set_transform("world", "odom", make_translation<World, Odom>(2.0, 0.0, 0.0));

    CHECK(call_count_1 == 2);
    CHECK(call_count_2 == 2);
}

TEST_CASE("ListenableTransformTree - callbacks for different edges") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");

    int world_odom_count = 0;
    int odom_base_count = 0;

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { world_odom_count++; });

    tree.on_update("odom", "base_link", [&](const concord::transform::GenericTransform &) { odom_base_count++; });

    CHECK(tree.edge_listener_count() == 2);

    // Only world->odom should trigger
    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));
    CHECK(world_odom_count == 1);
    CHECK(odom_base_count == 0);

    // Only odom->base_link should trigger
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 1.0, 0.0));
    CHECK(world_odom_count == 1);
    CHECK(odom_base_count == 1);
}

TEST_CASE("ListenableTransformTree - remove specific callback by ID") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count_1 = 0;
    int call_count_2 = 0;

    auto id1 = tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { call_count_1++; });

    auto id2 = tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { call_count_2++; });

    CHECK(tree.edge_listener_count() == 2);

    // Remove first callback
    tree.remove_listener(id1);
    CHECK(tree.edge_listener_count() == 1);

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));

    CHECK(call_count_1 == 0); // Should not be called
    CHECK(call_count_2 == 1); // Should be called

    // Remove second callback
    tree.remove_listener(id2);
    CHECK(tree.edge_listener_count() == 0);

    tree.set_transform("world", "odom", make_translation<World, Odom>(2.0, 0.0, 0.0));

    CHECK(call_count_1 == 0);
    CHECK(call_count_2 == 1); // Still 1, not triggered
}

TEST_CASE("ListenableTransformTree - remove all callbacks for an edge") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");

    int world_odom_count = 0;
    int odom_base_count = 0;

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { world_odom_count++; });

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { world_odom_count++; });

    tree.on_update("odom", "base_link", [&](const concord::transform::GenericTransform &) { odom_base_count++; });

    CHECK(tree.edge_listener_count() == 3);

    // Remove all callbacks for world->odom
    tree.remove_listeners("world", "odom");
    CHECK(tree.edge_listener_count() == 1);

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));
    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 1.0, 0.0));

    CHECK(world_odom_count == 0); // Should not be called
    CHECK(odom_base_count == 1);  // Should be called
}

TEST_CASE("ListenableTransformTree - on_any_update receives all updates") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");

    int any_count = 0;
    std::string last_to;
    std::string last_from;

    tree.on_any_update(
        [&](const std::string &to, const std::string &from, const concord::transform::GenericTransform &) {
            any_count++;
            last_to = to;
            last_from = from;
        });

    CHECK(tree.any_listener_count() == 1);

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));
    CHECK(any_count == 1);
    CHECK(last_to == "world");
    CHECK(last_from == "odom");

    tree.set_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 1.0, 0.0));
    CHECK(any_count == 2);
    CHECK(last_to == "odom");
    CHECK(last_from == "base_link");
}

TEST_CASE("ListenableTransformTree - callback receives correct transform data") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    concord::transform::GenericTransform received_tf;

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &tf) { received_tf = tf; });

    // Test with specific translation
    tree.set_transform("world", "odom", make_translation<World, Odom>(5.0, 10.0, 15.0));

    CHECK(received_tf.translation.x == doctest::Approx(5.0));
    CHECK(received_tf.translation.y == doctest::Approx(10.0));
    CHECK(received_tf.translation.z == doctest::Approx(15.0));

    // Test with rotation
    auto rot = concord::frame::Rotation<World, Odom>::from_euler_zyx(M_PI / 4.0, 0, 0);
    concord::frame::Transform<World, Odom> tf_with_rot{rot, dp::Point{1.0, 2.0, 3.0}};
    tree.set_transform("world", "odom", tf_with_rot);

    CHECK(received_tf.translation.x == doctest::Approx(1.0));
    CHECK(received_tf.translation.y == doctest::Approx(2.0));
    CHECK(received_tf.translation.z == doctest::Approx(3.0));
    // Check rotation is not identity
    CHECK(std::abs(received_tf.rotation.w - 1.0) > 0.01);
}

TEST_CASE("ListenableTransformTree - clear all listeners") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int edge_count = 0;
    int any_count = 0;

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { edge_count++; });

    tree.on_any_update(
        [&](const std::string &, const std::string &, const concord::transform::GenericTransform &) { any_count++; });

    CHECK(tree.edge_listener_count() == 1);
    CHECK(tree.any_listener_count() == 1);

    tree.clear_listeners();

    CHECK(tree.edge_listener_count() == 0);
    CHECK(tree.any_listener_count() == 0);

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));

    CHECK(edge_count == 0);
    CHECK(any_count == 0);
}

TEST_CASE("ListenableTransformTree - edge order independence") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count = 0;

    // Register with reverse order
    tree.on_update("odom", "world", [&](const concord::transform::GenericTransform &) { call_count++; });

    // Set transform with forward order should still trigger
    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));

    CHECK(call_count == 1);
}

TEST_CASE("ListenableTransformTree - clear also clears listeners") {
    concord::transform::ListenableTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count = 0;

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { call_count++; });

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0));
    CHECK(call_count == 1);

    tree.clear();
    CHECK(tree.edge_listener_count() == 0);

    // Re-register frames (they were cleared)
    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    tree.set_transform("world", "odom", make_translation<World, Odom>(2.0, 0.0, 0.0));
    CHECK(call_count == 1); // Should still be 1, listener was cleared
}

// ============================================================================
// ListenableTimedTransformTree Tests
// ============================================================================

TEST_CASE("ListenableTimedTransformTree - dynamic transform triggers callback") {
    concord::transform::ListenableTimedTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count = 0;
    dp::Point last_translation{0, 0, 0};

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &tf) {
        call_count++;
        last_translation = tf.translation;
    });

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 2.0, 3.0), 1.0);

    CHECK(call_count == 1);
    CHECK(last_translation.x == doctest::Approx(1.0));
    CHECK(last_translation.y == doctest::Approx(2.0));
    CHECK(last_translation.z == doctest::Approx(3.0));
}

TEST_CASE("ListenableTimedTransformTree - static transform triggers callback") {
    concord::transform::ListenableTimedTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count = 0;
    dp::Point last_translation{0, 0, 0};

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &tf) {
        call_count++;
        last_translation = tf.translation;
    });

    tree.set_static_transform("world", "odom", make_translation<World, Odom>(5.0, 6.0, 7.0));

    CHECK(call_count == 1);
    CHECK(last_translation.x == doctest::Approx(5.0));
    CHECK(last_translation.y == doctest::Approx(6.0));
    CHECK(last_translation.z == doctest::Approx(7.0));
}

TEST_CASE("ListenableTimedTransformTree - multiple updates trigger callbacks") {
    concord::transform::ListenableTimedTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count = 0;

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { call_count++; });

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 1.0);
    tree.set_transform("world", "odom", make_translation<World, Odom>(2.0, 0.0, 0.0), 2.0);
    tree.set_transform("world", "odom", make_translation<World, Odom>(3.0, 0.0, 0.0), 3.0);

    CHECK(call_count == 3);
}

TEST_CASE("ListenableTimedTransformTree - on_any_update for all transforms") {
    concord::transform::ListenableTimedTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");
    tree.register_frame<BaseLink>("base_link");

    std::vector<std::pair<std::string, std::string>> updates;

    tree.on_any_update([&](const std::string &to, const std::string &from,
                           const concord::transform::GenericTransform &) { updates.push_back({to, from}); });

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 1.0);
    tree.set_static_transform("odom", "base_link", make_translation<Odom, BaseLink>(0.0, 1.0, 0.0));

    CHECK(updates.size() == 2);
    CHECK(updates[0].first == "world");
    CHECK(updates[0].second == "odom");
    CHECK(updates[1].first == "odom");
    CHECK(updates[1].second == "base_link");
}

TEST_CASE("ListenableTimedTransformTree - remove listener works") {
    concord::transform::ListenableTimedTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count = 0;

    auto id = tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { call_count++; });

    tree.set_transform("world", "odom", make_translation<World, Odom>(1.0, 0.0, 0.0), 1.0);
    CHECK(call_count == 1);

    tree.remove_listener(id);

    tree.set_transform("world", "odom", make_translation<World, Odom>(2.0, 0.0, 0.0), 2.0);
    CHECK(call_count == 1); // Should still be 1
}

TEST_CASE("ListenableTimedTransformTree - clear clears listeners") {
    concord::transform::ListenableTimedTransformTree tree;

    tree.register_frame<World>("world");
    tree.register_frame<Odom>("odom");

    int call_count = 0;

    tree.on_update("world", "odom", [&](const concord::transform::GenericTransform &) { call_count++; });

    tree.on_any_update(
        [&](const std::string &, const std::string &, const concord::transform::GenericTransform &) { call_count++; });

    CHECK(tree.edge_listener_count() == 1);
    CHECK(tree.any_listener_count() == 1);

    tree.clear();

    CHECK(tree.edge_listener_count() == 0);
    CHECK(tree.any_listener_count() == 0);
}
