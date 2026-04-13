use thiserror::Error as ThisError;

#[derive(Debug, Clone, PartialEq, Eq, ThisError)]
pub enum Error {
    #[error("invalid argument: {0}")]
    InvalidArgument(String),
    #[error("out of range: {0}")]
    OutOfRange(String),
    #[error("not found: {0}")]
    NotFound(String),
    #[error("unsupported conversion: {0}")]
    UnsupportedConversion(String),
}

pub type Result<T> = std::result::Result<T, Error>;
