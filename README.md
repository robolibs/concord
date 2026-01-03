<img align="right" width="26%" src="./misc/logo.png">

# Concord

Type-safe coordinate transformations for robotics: WGS84, ECEF, UTM, ENU, NED with compile-time frame safety and datapod integration.

## Development Status

See [TODO.md](./TODO.md) for the complete development plan and current progress.

## Overview

Concord is a modern C++ coordinate transformation library designed for robotics applications. It provides geodetic conversions (WGS84 ↔ ECEF ↔ UTM) and local frame transformations (ENU, NED) with compile-time frame safety.

**Why Concord?**
- **Frame-safe transforms** - Compile-time validation prevents mixing incompatible coordinate frames
- **Self-contained local frames** - ENU/NED types extend `dp::Loc` and carry their reference origin, enabling round-trip conversions without external state
- **SE(3) transforms** - Full rigid-body transformations with composition, inverse, and Lie group operations
- **Transform trees** - Runtime frame graphs with BFS path finding and temporal interpolation
- **Zero-dependency geodesy** - WGS84, ECEF, UTM conversions without external libraries
- **Datapod backend** - Seamless integration with datapod's POD types for serialization

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                               CONCORD                                        │
├───────────────────────────┬─────────────────────────────────────────────────┤
│       EARTH REALM         │                  FRAME REALM                     │
│                           │                                                  │
│  ┌───────┐  ┌───────┐     │  ┌───────────┐  ┌───────────┐  ┌─────────────┐  │
│  │  WGS  │  │  ECF  │     │  │ ENU (Loc) │  │ NED (Loc) │  │ Transform   │  │
│  │ (Geo) │  │(Point)│     │  │ +origin   │  │ +origin   │  │  <To,From>  │  │
│  └───┬───┘  └───┬───┘     │  └─────┬─────┘  └─────┬─────┘  └──────┬──────┘  │
│      │          │         │        │              │               │         │
│  ┌───┴──────────┴───┐     │  ┌─────┴──────────────┴─────┐  ┌──────┴──────┐  │
│  │       UTM        │     │  │      FRD  /  FLU         │  │  Rotation   │  │
│  │     (dp::Utm)    │     │  │   (body frames, Point)   │  │  <To,From>  │  │
│  └──────────────────┘     │  └──────────────────────────┘  └─────────────┘  │
├───────────────────────────┴─────────────────────────────────────────────────┤
│                          TRANSFORM REALM                                     │
│  ┌──────────────────┐  ┌────────────────────┐  ┌─────────────────────────┐  │
│  │   FrameGraph     │  │   TransformTree    │  │  TimedTransformTree     │  │
│  │  (graphix-based) │  │  (BFS path find)   │  │  (temporal interpolate) │  │
│  └──────────────────┘  └────────────────────┘  └─────────────────────────┘  │
├─────────────────────────────────────────────────────────────────────────────┤
│                    convert(wgs).withRef(origin).to<ENU>()                    │
│                         Fluent conversion builder                            │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                            ┌───────▼────────┐
                            │    DATAPOD     │
                            │ dp::Loc/Geo/Utm│
                            └────────────────┘
```

## Installation

### Quick Start (CMake FetchContent)

```cmake
include(FetchContent)
FetchContent_Declare(
  concord
  GIT_REPOSITORY https://github.com/robolibs/concord
  GIT_TAG main
)
FetchContent_MakeAvailable(concord)

target_link_libraries(your_target PRIVATE concord::concord)
```

### Recommended: XMake

[XMake](https://xmake.io/) is a modern, fast, and cross-platform build system.

```lua
add_requires("concord")

target("your_target")
    set_kind("binary")
    add_packages("concord")
    add_files("src/*.cpp")
```

### Development Environment (Nix + Devbox)

```bash
cd concord
direnv allow    # Automatically loads environment
make config && make build && make test
```

## Usage

### Self-Contained Local Frames (dp::Loc Integration)

The key design feature: ENU and NED extend `dp::Loc`, carrying their reference origin internally. This enables round-trip conversions without external state:

```cpp
#include <concord/concord.hpp>
using namespace concord;

// Define reference origin
dp::Geo paris{48.8566, 2.3522, 35.0};

// Convert GPS waypoint to local ENU frame
earth::WGS waypoint{48.8570, 2.3530, 5.0};
frame::ENU local = frame::to_enu(paris, waypoint);

// ENU carries its origin - access it anytime
std::cout << "Origin: " << local.origin.latitude << "\n";

// Convert back to WGS - NO external reference needed!
earth::WGS back = frame::to_wgs(local);  // Self-contained!

// Use semantic accessors
std::cout << "East: " << local.east() << "m, "
          << "North: " << local.north() << "m\n";
```

### Fluent Conversion Builder

Chain conversions with automatic reference propagation:

```cpp
// WGS -> ENU requires reference
auto enu = convert(waypoint).withRef(paris).to<frame::ENU>().build();

// ENU -> WGS needs NO reference (ENU carries it!)
auto wgs = convert(enu.value()).to<earth::WGS>().build();

// Chain multiple conversions
auto result = convert(gps_pos)
    .withRef(datum)
    .to<frame::ENU>()
    .to<frame::NED>()
    .to<earth::WGS>()
    .build();
```

### Datapod Interoperability

Concord types extend datapod types - zero-copy interop:

```cpp
// dp::Geo <-> earth::WGS (same memory layout)
dp::Geo geo{48.8566, 2.3522, 35.0};
earth::WGS wgs{geo};  // Zero-copy construction
const dp::Geo& ref = wgs;  // Implicit conversion

// dp::Loc <-> frame::ENU/NED (ENU IS-A dp::Loc)
dp::Loc robot_loc{dp::Point{100, 50, 5}, geo};
frame::ENU enu{robot_loc};
double dist = enu.distance_from_origin();  // dp::Loc method!

// dp::Point <-> frame::FRD/FLU (body frames)
dp::Point offset{0.5, 0.0, 0.3};
frame::FLU sensor{offset};
double mag = sensor.magnitude();  // dp::Point method!
```

### SE(3) Transforms and Frame Trees

Build robot TF trees with compile-time frame safety:

```cpp
using namespace concord::frame;

// Define frame tags (empty structs)
struct World {};
struct Body {};
struct Camera {};

// Create transforms
auto T_world_body = Transform<World, Body>::from_qt(
    dp::Quaternion{0.707, 0.0, 0.0, 0.707},
    dp::Point{10.0, 5.0, 0.0}
);

auto T_body_camera = Transform<Body, Camera>::from_qt(
    dp::Quaternion{1.0, 0.0, 0.0, 0.0},
    dp::Point{0.3, 0.0, 0.5}
);

// Compose: T_world_camera = T_world_body * T_body_camera
auto T_world_camera = T_world_body * T_body_camera;

// Transform points
dp::Point p_camera{2.0, 0.0, 0.0};
dp::Point p_world = T_world_camera * p_camera;
```

### Runtime Transform Tree with Path Finding

```cpp
transform::TransformTree tree;

// Register frames
tree.register_frame<World>("world");
tree.register_frame<Body>("base_link");
tree.register_frame<Camera>("camera");

// Set transforms
tree.set_transform("base_link", "camera", T_body_camera);
tree.set_transform("world", "base_link", T_world_body);

// Lookup ANY transform (automatic path finding via BFS)
auto tf = tree.lookup("world", "camera");
if (tf) {
    dp::Point p = tf->apply(dp::Point{1, 0, 0});
}

// Get path for debugging
auto path = tree.get_path("world", "camera");
// -> ["world", "base_link", "camera"]
```

### Temporal Transform Tree (Time-Interpolated Lookups)

```cpp
transform::TimedTransformTree tree;

tree.register_frame<World>("world");
tree.register_frame<Body>("base_link");

// Add timestamped transforms
tree.set_transform("world", "base_link", tf1, 1.0);  // t=1.0s
tree.set_transform("world", "base_link", tf2, 2.0);  // t=2.0s

// Lookup with interpolation
auto tf = tree.lookup("world", "base_link", 1.5);  // Interpolated!

// Static transforms (no time history)
tree.set_static("base_link", "camera", T_body_camera);
```

### Lie Group Operations

```cpp
using namespace concord::frame;

// Exponential/logarithmic maps
RotationTangent omega{0.1, 0.2, 0.3};  // axis-angle
auto rot = exp<World, Body>(omega);
auto back = log(rot);

// Geodesic interpolation (SLERP)
auto mid = slerp(rot1, rot2, 0.5);

// Karcher mean of multiple rotations
std::vector<Rotation<World, Body>> rotations = {...};
auto mean = average(std::span{rotations});

// Spline interpolation through transforms
TransformSpline<World, Body> spline;
spline.add_point(tf1);
spline.add_point(tf2);
spline.add_point(tf3);
spline.build();
auto smooth = spline.evaluate_normalized(0.5);
```

### Batch Conversions

```cpp
// SIMD-accelerated batch conversion
std::vector<earth::WGS> gps_points = {...};
dp::Vector<frame::ENU> enu_batch = earth::batch_to_enu(origin, gps_points);

// Each ENU carries the shared origin
for (const auto& enu : enu_batch) {
    earth::WGS wgs = frame::to_wgs(enu);  // Self-contained!
}
```

## Features

- **Frame Tags & Compile-Time Safety** - `FrameTag::LocalTangent`, `Body`, `Geocentric`
  ```cpp
  static_assert(ENU::tag == FrameTag::LocalTangent);
  static_assert(FRD::tag == FrameTag::Body);
  ```

- **Semantic Accessors** - Self-documenting code for robotics
  ```cpp
  enu.east(), enu.north(), enu.up()
  ned.north(), ned.east(), ned.down()
  frd.forward(), frd.right(), frd.down()  // Aerospace (PX4)
  flu.forward(), flu.left(), flu.up()     // ROS convention
  ```

- **frame_cast<To>(from)** - Zero-cost axis shuffling between co-located frames
  ```cpp
  NED ned = frame_cast<NED>(enu);  // Just shuffles axes, preserves origin
  FLU flu = frame_cast<FLU>(frd);  // Aerospace → ROS convention
  ```

- **Transform Composition** - Build TF trees naturally
  ```cpp
  auto T_AC = T_AB * T_BC;           // Compose transforms
  auto T_BA = T_AB.inverse();        // Invert transform
  auto p_world = T_world_body * p;   // Transform points
  ```

- **Unit Conversion** - Degrees or radians, your choice
  ```cpp
  wgs.latitude;                           // degrees (default)
  wgs.lat_rad();                          // radians
  auto pos = earth::WGS::from_radians(lat_rad, lon_rad, alt);
  ```

- **Stream Output** - Debug-friendly printing
  ```cpp
  std::cout << wgs;  // WGS{lat=48.86deg, lon=2.35deg, alt=35m}
  std::cout << enu;  // ENU{e=10, n=20, u=5 @(48.86,2.35)}
  std::cout << tf;   // Transform{Rotation{q=[1,0,0,0]}, t=[10,5,0]}
  ```

- **Rotation Factories** - Multiple ways to create rotations
  ```cpp
  auto r1 = Rotation<A,B>::from_euler_zyx(yaw, pitch, roll);
  auto r2 = Rotation<A,B>::from_axis_angle(axis, angle);
  auto r3 = Rotation<A,B>::from_matrix(R);
  auto r4 = Rotation<A,B>::identity();
  ```

- **Spline Interpolation** - C1-smooth trajectories through transforms
  ```cpp
  TimedTransformSpline<World, Body> spline;
  spline.add_point(t1, tf1);
  spline.add_point(t2, tf2);
  spline.build();
  auto tf = spline.evaluate_at(query_time);
  ```

## API Reference

### Earth Realm (`concord::earth`)

| Type | Base | Description |
|------|------|-------------|
| `WGS` | `dp::Geo` | WGS84 geodetic (lat/lon/alt) |
| `ECF` | `dp::Point` | Earth-Centered Fixed (ECEF) |
| `UTM` | `dp::Utm` | Universal Transverse Mercator |

### Frame Realm (`concord::frame`)

| Type | Base | Description |
|------|------|-------------|
| `ENU` | `dp::Loc` | East-North-Up (carries origin!) |
| `NED` | `dp::Loc` | North-East-Down (carries origin!) |
| `FRD` | `dp::Point` | Forward-Right-Down (body, PX4) |
| `FLU` | `dp::Point` | Forward-Left-Up (body, ROS) |
| `Transform<To,From>` | - | SE(3) rigid body transform |
| `Rotation<To,From>` | - | SO(3) rotation |

### Transform Realm (`concord::transform`)

| Type | Description |
|------|-------------|
| `FrameGraph` | Graph storing frames and transforms |
| `TransformTree` | High-level tree with BFS path finding |
| `TimedTransformTree` | Temporal transforms with interpolation |
| `TimedTransformBuffer` | Circular buffer for transform history |
| `GenericTransform` | Type-erased transform for runtime composition |

### Key Functions

| Function | Description |
|----------|-------------|
| `earth::to_ecf(WGS)` | WGS84 → ECEF |
| `earth::to_wgs(ECF)` | ECEF → WGS84 |
| `earth::to_utm(WGS)` | WGS84 → UTM (returns `dp::Result`) |
| `frame::to_enu(ref, WGS)` | WGS84 → local ENU with origin |
| `frame::to_ned(ref, WGS)` | WGS84 → local NED with origin |
| `frame::to_wgs(ENU/NED)` | Local → WGS84 (self-contained!) |
| `frame::frame_cast<To>(from)` | Zero-cost axis shuffle |
| `earth::batch_to_enu(ref, span)` | SIMD batch conversion |

### Lie Group Functions (`concord::frame`)

| Function | Description |
|----------|-------------|
| `exp<To,From>(omega)` | Tangent → Rotation (exponential map) |
| `log(rotation)` | Rotation → Tangent (logarithmic map) |
| `slerp(r1, r2, t)` | Spherical linear interpolation |
| `interpolate(tf1, tf2, t)` | Geodesic interpolation for SE(3) |
| `average(span<Rotation>)` | Karcher/Frechet mean |
| `left_jacobian_so3(omega)` | Left Jacobian for optimization |

## Examples

See the `examples/` directory:
- `datapod_interop_example.cpp` - dp::Loc/Geo/Utm interoperability
- `convert_builder_example.cpp` - Fluent API with self-contained ENU
- `robot_frame_tree.cpp` - Transform tree with path finding
- `wgs_to_enu_example.cpp` - GPS to local frame conversion
- `wgs_to_utm_example.cpp` - UTM projection for mapping

## License

MIT License - see [LICENSE](./LICENSE) for details.

## Acknowledgments

- **datapod** - POD types backend (dp::Loc, dp::Geo, dp::Utm, dp::Point, dp::Quaternion)
- **optinum** - SIMD operations and Lie group math
- **graphix** - Graph algorithms for transform tree path finding
