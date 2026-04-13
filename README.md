# concord_rs

`concord` is a Rust coordinate and frame transformation library for robotics.

Current scope:

- WGS84, ECF, and UTM conversions
- ENU and NED local tangent frames with embedded reference origin
- Typed `Rotation<To, From>` and `Transform<To, From>`
- Runtime transform lookup with `TransformTree`
- Temporal transform lookup with `TimedTransformTree`
- Practical Lie helpers and spline helpers on top of `glam`
- C ABI in [`include/concord.h`](include/concord.h)
- Python bindings via `maturin` in [`pyproject.toml`](pyproject.toml)

The crate uses:

- `glam` for vectors, quaternions, and matrices
- `graphix_rs` as the local graph backend for runtime transform lookup

Status:

- local-only for now
- not configured for crates.io publication while `graphix` remains a local path dependency

## Install

```toml
[dependencies]
concord = { path = "../concord_rs" }
```

This assumes `../concord_rs` and `../graphix_rs` stay as sibling local crates, matching the current workspace layout.

## Examples

### WGS to ENU

```rust
use concord::{Geo, Wgs, convert, frame::Enu};

let origin = Geo::new(52.0, 4.0, 10.0);
let point = Wgs::new(52.0001, 4.0002, 12.0);

let enu = convert(point)
    .with_ref(origin)
    .to::<Enu>()
    .build()
    .expect("valid ENU conversion");

assert_eq!(enu.origin, origin);
```

### Builder chains

```rust
use concord::{Geo, Wgs, convert, frame::{Enu, Ned}};

let origin = Geo::new(48.8566, 2.3522, 35.0);
let point = Wgs::new(48.8570, 2.3530, 40.0);

let roundtrip = convert(point)
    .with_ref(origin)
    .to::<Enu>()
    .to::<Ned>()
    .to::<Wgs>()
    .build()
    .expect("roundtrip");

assert!((roundtrip.latitude - point.latitude).abs() < 1e-10);
```

### Transform tree lookup

```rust
use glam::{DQuat, DVec3};
use concord::{Transform, TransformTree};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Base;
#[derive(Debug, Clone, Copy)]
struct Camera;

let mut tree = TransformTree::new();
tree.register_frame::<World>("world");
tree.register_frame::<Base>("base");
tree.register_frame::<Camera>("camera");

tree.set_transform(
    "world",
    "base",
    Transform::<World, Base>::from_qt(DQuat::IDENTITY, DVec3::new(1.0, 2.0, 0.0)),
);
tree.set_transform(
    "base",
    "camera",
    Transform::<Base, Camera>::from_qt(DQuat::IDENTITY, DVec3::new(0.0, 0.0, 1.0)),
);

let tf = tree.lookup("world", "camera").expect("path exists");
assert_eq!(tf.translation, DVec3::new(1.0, 2.0, 1.0));
```

### Timed transform lookup

```rust
use glam::{DQuat, DVec3};
use concord::{TimedTransformTree, Transform};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Odom;

let mut tree = TimedTransformTree::new();
tree.register_frame::<World>("world");
tree.register_frame::<Odom>("odom");

tree.set_transform(
    "world",
    "odom",
    Transform::<World, Odom>::from_qt(DQuat::IDENTITY, DVec3::ZERO),
    1.0,
);
tree.set_transform(
    "world",
    "odom",
    Transform::<World, Odom>::from_qt(DQuat::IDENTITY, DVec3::new(10.0, 0.0, 0.0)),
    2.0,
);

let tf = tree.lookup("world", "odom", 1.5).expect("interpolated transform");
assert_eq!(tf.translation, DVec3::new(5.0, 0.0, 0.0));
```

Runnable versions of these snippets live in [`examples/`](examples).

## C ABI

The C ABI mirrors the stable non-generic surface of the crate:

- WGS, ECF, UTM, ENU, and NED conversions
- ENU/NED casts
- a named-frame runtime transform tree

The public header lives at [`include/concord.h`](include/concord.h).

## Python

The Python module is packaged with `maturin`, following the same pattern used in `../maptrax_rs`.

Typical commands:

```bash
cargo check --features python
maturin develop --features python-extension
```

The Python surface currently exposes:

- top-level conversion helpers like `wgs_to_ecf`, `wgs_to_utm`, and `wgs_to_enu`
- a `TransformTree` class for named runtime transforms
