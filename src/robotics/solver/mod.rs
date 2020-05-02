mod constraint;
mod fast_ldlt;
mod lcp_solver;
mod dantzig_lcp_solver;

pub enum ConstraintSolverType {
    MlcpSolver,
    MultibodySolver,
}


trait ConstraintSolver
{
    fn prepare_solve(num_bodies: usize, num_manifolds: usize);

    ///solve a group of constraints
    fn solve_group();

    fn all_solved();

    ///clear internal cached data and reset random seed
    fn reset();

    fn get_solver_type() -> ConstraintSolverType;
}

trait SequentialImpulseConstraintSolver : ConstraintSolver {

}