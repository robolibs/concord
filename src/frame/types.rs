use datapod::Point;

use crate::earth::Geo;
use crate::frame::tags::{FrameTag, FrameTraits};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Enu {
    pub local: Point,
    pub origin: Geo,
}

impl Enu {
    pub const NAME: &str = "enu";
    pub const TAG: FrameTag = FrameTag::LocalTangent;

    pub fn new(east: f64, north: f64, up: f64, origin: Geo) -> Self {
        Self {
            local: Point::new(east, north, up),
            origin,
        }
    }

    pub fn east(self) -> f64 {
        self.local.x
    }

    pub fn north(self) -> f64 {
        self.local.y
    }

    pub fn up(self) -> f64 {
        self.local.z
    }

    pub fn x(self) -> f64 {
        self.local.x
    }

    pub fn y(self) -> f64 {
        self.local.y
    }

    pub fn z(self) -> f64 {
        self.local.z
    }

    pub fn point(self) -> Point {
        self.local
    }

    pub fn ref_origin(self) -> Geo {
        self.origin
    }

    pub fn distance_from_origin(self) -> f64 {
        self.local.magnitude()
    }

    pub fn distance_from_origin_2d(self) -> f64 {
        (self.local.x * self.local.x + self.local.y * self.local.y).sqrt()
    }

    pub fn same_origin(self, other: Self) -> bool {
        self.origin == other.origin
    }

    pub fn distance_to(self, other: Self) -> f64 {
        self.local.distance_to(other.local)
    }

    pub fn distance_to_2d(self, other: Self) -> f64 {
        self.local.distance_to_2d(other.local)
    }
}

impl FrameTraits for Enu {
    const NAME: &'static str = Self::NAME;
    const TAG: FrameTag = Self::TAG;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ned {
    pub local: Point,
    pub origin: Geo,
}

impl Ned {
    pub const NAME: &str = "ned";
    pub const TAG: FrameTag = FrameTag::LocalTangent;

    pub fn new(north: f64, east: f64, down: f64, origin: Geo) -> Self {
        Self {
            local: Point::new(north, east, down),
            origin,
        }
    }

    pub fn north(self) -> f64 {
        self.local.x
    }

    pub fn east(self) -> f64 {
        self.local.y
    }

    pub fn down(self) -> f64 {
        self.local.z
    }

    pub fn x(self) -> f64 {
        self.local.x
    }

    pub fn y(self) -> f64 {
        self.local.y
    }

    pub fn z(self) -> f64 {
        self.local.z
    }

    pub fn point(self) -> Point {
        self.local
    }

    pub fn ref_origin(self) -> Geo {
        self.origin
    }

    pub fn distance_from_origin(self) -> f64 {
        self.local.magnitude()
    }

    pub fn distance_from_origin_2d(self) -> f64 {
        (self.local.x * self.local.x + self.local.y * self.local.y).sqrt()
    }

    pub fn same_origin(self, other: Self) -> bool {
        self.origin == other.origin
    }
}

impl FrameTraits for Ned {
    const NAME: &'static str = Self::NAME;
    const TAG: FrameTag = Self::TAG;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frd {
    pub local: Point,
}

impl Frd {
    pub const NAME: &str = "frd";
    pub const TAG: FrameTag = FrameTag::Body;

    pub fn new(forward: f64, right: f64, down: f64) -> Self {
        Self {
            local: Point::new(forward, right, down),
        }
    }

    pub fn forward(self) -> f64 {
        self.local.x
    }

    pub fn right(self) -> f64 {
        self.local.y
    }

    pub fn down(self) -> f64 {
        self.local.z
    }

    pub fn x(self) -> f64 {
        self.local.x
    }

    pub fn y(self) -> f64 {
        self.local.y
    }

    pub fn z(self) -> f64 {
        self.local.z
    }

    pub fn point(self) -> Point {
        self.local
    }

    pub fn magnitude(self) -> f64 {
        self.local.magnitude()
    }
}

impl FrameTraits for Frd {
    const NAME: &'static str = Self::NAME;
    const TAG: FrameTag = Self::TAG;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Flu {
    pub local: Point,
}

impl Flu {
    pub const NAME: &str = "flu";
    pub const TAG: FrameTag = FrameTag::Body;

    pub fn new(forward: f64, left: f64, up: f64) -> Self {
        Self {
            local: Point::new(forward, left, up),
        }
    }

    pub fn forward(self) -> f64 {
        self.local.x
    }

    pub fn left(self) -> f64 {
        self.local.y
    }

    pub fn up(self) -> f64 {
        self.local.z
    }

    pub fn x(self) -> f64 {
        self.local.x
    }

    pub fn y(self) -> f64 {
        self.local.y
    }

    pub fn z(self) -> f64 {
        self.local.z
    }

    pub fn point(self) -> Point {
        self.local
    }

    pub fn magnitude(self) -> f64 {
        self.local.magnitude()
    }
}

impl FrameTraits for Flu {
    const NAME: &'static str = Self::NAME;
    const TAG: FrameTag = Self::TAG;
}
