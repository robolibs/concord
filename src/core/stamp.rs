#[derive(Debug, Clone, PartialEq)]
pub struct Stamp<T> {
    pub timestamp: i64,
    pub value: T,
}

impl<T> Stamp<T> {
    pub const fn new(timestamp: i64, value: T) -> Self {
        Self { timestamp, value }
    }
}
