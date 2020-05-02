use crate::math::*;

pub struct ConstraintInfo {
    /// Impulse
    x: Vec<Scalar>,

    /// Lower bound of x
    lo: Vec<Scalar>,

    /// Upper bound of x
    hi: Vec<Scalar>,

    /// Bias term
    b: Vec<Scalar>,

    /// Slack variable
    w: Vec<Scalar>,

    /// Friction index
    findex: Vec<usize>,

    /// Inverse of time step
    inv_time_step: Scalar,
}