# Concord TODO

Roadmap for making Concord a complete coordinate/transform library for robotics without runtime dependencies.

---

## Priority 1: Essential

### [ ] `tree/` - Frame Graph / Transform Tree
The "TF tree" concept without pub/sub runtime. Just math and graph traversal.

```cpp
namespace concord::tree {
    class FrameTree {
        void set(const std::string& parent, const std::string& child, Transform tf);
        auto lookup(const std::string& from, const std::string& to) -> Result<Transform>;
    };
}
```

Features:
- [ ] String-based frame IDs for runtime flexibility
- [ ] Automatic transform chaining (world -> base -> arm -> gripper)
- [ ] Inverse lookup (automatic inversion when traversing backwards)
- [ ] Cycle detection
- [ ] Optional: compile-time frame graph with type-based IDs
- [ ] Optional: YAML/JSON serialization for robot configs

---
---

## Architecture

```
concord/
├── earth/          [done] Geodetic (WGS84, ECEF, UTM, ENU, NED)
├── frame/          [done] SE(3) transforms, frame types, cast
├── tree/           [todo] Frame graph / TF tree
```

---
