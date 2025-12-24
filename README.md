<img align="right" width="26%" src="./misc/logo.png">

# Concord

Type-safe coordinate transformations for robotics: WGS84, ECEF, UTM, ENU, NED with compile-time frame safety.

## Development Status

See [PLAN.md](./PLAN.md) for the complete development plan and current progress.

## Overview

Concord is a modern C++ coordinate transformation library designed for robotics applications. It provides geodetic conversions (WGS84 ↔ ECEF ↔ UTM) and local frame transformations (ENU, NED) with compile-time frame safety inspired by the refx library pattern.

**Why Concord for Robotics?**
- **Frame-safe transforms** - Compile-time validation prevents mixing incompatible coordinate frames
- **Robotics-native frames** - Built-in support for ENU, NED, FRD (aerospace), FLU (ROS) conventions
- **SE(3) transforms** - Full rigid-body transformations with composition and inverse
- **Zero-dependency geodesy** - WGS84, ECEF, UTM conversions without external libraries
- **Datapod backend** - Seamless integration with datapod's POD types for serialization

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                            CONCORD                                   │
├─────────────────────────────┬───────────────────────────────────────┤
│         EARTH REALM         │            FRAME REALM                │
│                             │                                       │
│  ┌───────┐  ┌───────┐      │  ┌───────┐  ┌───────┐  ┌───────────┐  │
│  │  WGS  │  │  ECF  │      │  │  ENU  │  │  NED  │  │ Transform │  │
│  │ (LLA) │  │(ECEF) │      │  │       │  │       │  │  <To,From>│  │
│  └───┬───┘  └───┬───┘      │  └───┬───┘  └───┬───┘  └─────┬─────┘  │
│      │          │          │      │          │            │        │
│  ┌───┴──────────┴───┐      │  ┌───┴──────────┴───┐  ┌─────┴─────┐  │
│  │       UTM        │      │  │   FRD  /  FLU    │  │  Rotation │  │
│  │   (projections)  │      │  │  (body frames)   │  │  <To,From>│  │
│  └──────────────────┘      │  └──────────────────┘  └───────────┘  │
├─────────────────────────────┴───────────────────────────────────────┤
│                         frame_cast<To>(from)                        │
│                    Zero-cost axis shuffling (ENU↔NED)               │
├─────────────────────────────────────────────────────────────────────┤
│                      convert(wgs).withDatum(d).to<ENU>()            │
│                         Fluent conversion builder                   │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                            ┌───────▼────────┐
                            │    DATAPOD     │
                            │ dp::Point/Quat │
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

### Basic: Geodetic Conversions

```cpp
#include <concord/concord.hpp>
using namespace concord;

// WGS84 coordinates (robot's GPS position)
earth::WGS gps_pos{48.8566, 2.3522, 35.0};  // Paris

// Convert to ECEF for sensor fusion
earth::ECF ecef = earth::to_ecf(gps_pos);

// Convert to UTM for map-based navigation
auto utm = earth::to_utm(gps_pos);  // dp::Result<earth::UTM>
std::cout << utm.value();  // UTM{zone=31N, e=448251m, n=5411935m, alt=35m}
```

### Local Frame Transforms (ENU/NED)

```cpp
// Define datum at robot's starting position
frame::Datum datum{earth::WGS{48.8566, 2.3522, 0.0}};

// Convert GPS waypoint to local ENU frame
earth::WGS waypoint{48.8570, 2.3530, 5.0};
frame::ENU local = frame::to_enu(datum, waypoint);

// Use semantic accessors
std::cout << "East: " << local.east() << "m, "
          << "North: " << local.north() << "m, "
          << "Up: " << local.up() << "m\n";

// Switch to NED (for PX4/ArduPilot compatibility)
frame::NED ned = frame::frame_cast<NED>(local);  // Zero-cost!
```

### SE(3) Transforms (Sensor Fusion, TF Trees)

```cpp
using namespace concord::frame;

// Define frame tags for your robot
struct World {};
struct Body {};
struct Camera {};

// Body pose in world frame (from odometry/SLAM)
auto T_world_body = Transform<World, Body>::from_qt(
    dp::Quaternion{0.707, 0.0, 0.0, 0.707},  // 90° yaw
    dp::Point{10.0, 5.0, 0.0}                 // position
);

// Camera mounted on robot body
auto T_body_camera = Transform<Body, Camera>::from_qt(
    dp::Quaternion{1.0, 0.0, 0.0, 0.0},  // no rotation
    dp::Point{0.3, 0.0, 0.5}              // 30cm forward, 50cm up
);

// Compose: get camera pose in world frame
auto T_world_camera = T_world_body * T_body_camera;

// Transform detected object from camera frame to world
dp::Point object_in_camera{2.0, 0.0, 0.0};  // 2m in front of camera
dp::Point object_in_world = T_world_camera * object_in_camera;
```

### Fluent Conversion Builder

```cpp
auto result = convert(gps_pos)
    .withDatum(datum)
    .to<frame::ENU>()
    .to<frame::NED>()
    .to<earth::WGS>()
    .build();  // dp::Result<earth::WGS>

if (result.is_ok()) {
    std::cout << result.value() << "\n";
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
  NED ned = frame_cast<NED>(enu);  // Just shuffles axes, no math
  FLU flu = frame_cast<FLU>(frd);  // Aerospace → ROS convention
  ```

- **Unit Conversion** - Degrees or radians, your choice
  ```cpp
  wgs.latitude();                      // degrees (default)
  wgs.latitude(earth::AngleUnit::Rad); // radians
  auto pos = earth::WGS::from_radians(lat_rad, lon_rad, alt);
  ```

- **Transform Composition** - Build TF trees naturally
  ```cpp
  auto T_AC = T_AB * T_BC;           // Compose
  auto T_BA = T_AB.inverse();        // Invert
  auto p_world = T_world_body * p;   // Transform points
  ```

- **Stream Output** - Debug-friendly printing
  ```cpp
  std::cout << wgs;  // WGS{lat=48.86deg, lon=2.35deg, alt=35m}
  std::cout << enu;  // ENU{e=10, n=20, u=5}
  std::cout << tf;   // Transform{Rotation{q=[1,0,0,0]}, t=[10,5,0]}
  ```

## API Reference

### Earth Realm (`concord::earth`)

| Type | Description |
|------|-------------|
| `WGS` | WGS84 geodetic (lat/lon/alt) |
| `ECF` | Earth-Centered Fixed (ECEF) |
| `UTM` | Universal Transverse Mercator |

| Function | Description |
|----------|-------------|
| `to_ecf(WGS)` | WGS84 → ECEF |
| `to_wgs(ECF)` | ECEF → WGS84 |
| `to_utm(WGS)` | WGS84 → UTM (returns `dp::Result`) |
| `to_wgs(UTM)` | UTM → WGS84 (returns `dp::Result`) |

### Frame Realm (`concord::frame`)

| Type | Description |
|------|-------------|
| `ENU` | East-North-Up (local tangent) |
| `NED` | North-East-Down (aerospace) |
| `FRD` | Forward-Right-Down (body, PX4) |
| `FLU` | Forward-Left-Up (body, ROS) |
| `Datum` | Origin for local frames |
| `Transform<To,From>` | SE(3) rigid body transform |
| `Rotation<To,From>` | SO(3) rotation |

| Function | Description |
|----------|-------------|
| `to_enu(Datum, WGS)` | WGS84 → local ENU |
| `to_ned(Datum, WGS)` | WGS84 → local NED |
| `to_wgs(Datum, ENU/NED)` | Local → WGS84 |
| `frame_cast<To>(from)` | Zero-cost axis shuffle |

## Examples

See the `examples/` directory:
- `wgs_to_enu_example.cpp` - GPS to local frame conversion
- `wgs_to_utm_example.cpp` - UTM projection for mapping
- `convert_builder_example.cpp` - Fluent API chaining

## License

MIT License - see [LICENSE](./LICENSE) for details.

## Acknowledgments

- **datapod** - POD types backend (Point, Quaternion, Result)
- **refx** - Inspiration for frame-safe transform design
