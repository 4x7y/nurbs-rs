use crate::math::*;

/// min/max range to check the tmp position
#[derive(Copy, Debug, Clone)]
pub struct Range {
    pub min: Scalar,
    pub max: Scalar,
}

impl Range {

    /// Create new Range instance
    pub fn new(min: Scalar, max: Scalar) -> Self {
        Range { min, max }
    }

    /// Check if the value is in the range
    pub fn is_valid(&self, val: Scalar) -> bool {
        val <= self.max && val >= self.min
    }
}