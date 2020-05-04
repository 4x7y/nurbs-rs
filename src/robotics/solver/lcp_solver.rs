pub use crate::math::Scalar;

pub trait LCPSolver {
    /// Solve constraint impulses for a constrained group
    fn solve(&self);

    /// Set time step
    fn set_time_step(&mut self, time_step: Scalar);

    /// Return time step
    fn get_time_step(&self) -> Scalar;
}

/// Round an integer up to a multiple of 4, except that 0 and 1 are
/// unmodified (used to compute matrix leading dimensions)
fn pad(a: usize) -> usize {
    if a > 1 {
        ((a-1)|3) + 1
    } else {
        a
    }
}

fn factor_ldlt() {

}

fn solve_ldlt() {

}