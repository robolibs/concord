//! Rust coordinate and frame transformation library for robotics.
//!
//! The crate provides:
//!
//! - WGS84, ECF, and UTM conversions
//! - ENU and NED local tangent frames with embedded reference origins
//! - typed `Rotation<To, From>` and `Transform<To, From>`
//! - runtime and temporal transform lookup
//!
//! # Examples
//!
//! Convert WGS coordinates into a local ENU frame:
//!
//! ```rust
//! use concord::{Geo, Wgs, convert, frame::Enu};
//!
//! let origin = Geo::new(52.0, 4.0, 10.0);
//! let point = Wgs::new(52.0001, 4.0002, 12.0);
//!
//! let enu = convert(point)
//!     .with_ref(origin)
//!     .to::<Enu>()
//!     .build()
//!     .expect("valid ENU conversion");
//!
//! assert_eq!(enu.origin, origin);
//! ```
//!
//! Compose runtime transforms through the transform tree:
//!
//! ```rust
//! use glam::{DQuat, DVec3};
//! use concord::{Transform, TransformTree};
//!
//! #[derive(Debug, Clone, Copy)]
//! struct World;
//! #[derive(Debug, Clone, Copy)]
//! struct Base;
//! #[derive(Debug, Clone, Copy)]
//! struct Camera;
//!
//! let mut tree = TransformTree::new();
//! tree.register_frame::<World>("world");
//! tree.register_frame::<Base>("base");
//! tree.register_frame::<Camera>("camera");
//!
//! tree.set_transform(
//!     "world",
//!     "base",
//!     Transform::<World, Base>::from_qt(DQuat::IDENTITY, DVec3::new(1.0, 2.0, 0.0)),
//! );
//! tree.set_transform(
//!     "base",
//!     "camera",
//!     Transform::<Base, Camera>::from_qt(DQuat::IDENTITY, DVec3::new(0.0, 0.0, 1.0)),
//! );
//!
//! let tf = tree.lookup("world", "camera").expect("path exists");
//! assert_eq!(tf.translation, DVec3::new(1.0, 2.0, 1.0));
//! ```

pub mod core;
pub mod earth;
pub mod ffi;
pub mod frame;
#[cfg(feature = "python")]
pub mod python;
pub mod transform;

pub use core::{Error, Result, Stamp};
pub use earth::{
    Ecf, Geo, Utm, Wgs, batch_to_ecf, batch_to_enu, batch_to_ned, batch_to_utm, batch_to_wgs,
    batch_to_wgs_from_enu, batch_to_wgs_from_ned, r_enu_from_ecf, r_ned_from_ecf, to_ecf, to_utm,
    to_wgs, to_wgs_optimized, utm_to_wgs,
};
pub use frame::{
    ConvertBuilder, ConvertInto, Enu, Flu, FrameCast, Frd, Ned, Rotation, RotationSpline,
    TimedRotationSpline, TimedTransformSpline, Transform, TransformSpline, convert, enu_to_ned,
    flu_to_frd, frame_cast, frd_to_flu, ned_to_enu, sample_rotation_spline,
    sample_transform_spline, to_enu, to_ned, to_wgs_from_enu, to_wgs_from_ned,
};
pub use transform::{
    FrameGraph, FrameInfo, GenericTransform, ListenableTimedTransformTree, ListenableTransformTree,
    TimedTransformBuffer, TimedTransformTree, TransformTree, interpolate, lerp, slerp,
};
