#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameTag {
    Geocentric,
    LocalTangent,
    Body,
    Sensor,
}

pub trait FrameTraits {
    const NAME: &'static str;
    const TAG: FrameTag;
}
