use crate::robotics::solver::lcp_solver::LCPSolver;
use crate::math::{MatrixDDf, VectorDf, Scalar};

/// DantzigLCPSolver is a LCP solver that uses ODE's implementation
/// of Dantzig algorithm
pub struct DantzigLCPSolver {
    time_step: Scalar,
}

impl LCPSolver for DantzigLCPSolver {
    fn solve(&self) {
        unimplemented!()
    }

    fn set_time_step(&mut self, time_step: Scalar) {
        self.time_step = time_step;
    }

    fn get_time_step(&self) -> Scalar {
        self.time_step
    }
}

impl DantzigLCPSolver {

    fn is_symmetric(A: &MatrixDDf) -> bool {
        unimplemented!()
    }

    fn is_symmetric_block(A: &MatrixDDf, begin: usize, end: usize) -> bool {
        unimplemented!()
    }

    fn solve_lcp(
        n: usize,
        A: &MatrixDDf,
        x: &VectorDf,
        b: &VectorDf,
        w: &VectorDf,
        nub: usize,
        lo: &VectorDf,
        hi: &VectorDf,
        f_index: &Vec<usize>,
        early_termination: bool,
    ) -> bool {

        assert!(n > 0 && nub <= n);
        for k in 0..n {
            assert!(lo[k] <= 0. && hi[k] >= 0.);
        }

        unimplemented!()
    }
}