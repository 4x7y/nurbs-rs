use na::*;
use na::allocator::Allocator;
use crate::math::matrix::{VectorNf, MatrixMNf};
use crate::utils::common::{zeros, zeros_3d};

pub struct JointSpaceTrajectory<D: Dim + DimName>
    where DefaultAllocator: Allocator<f32, D> {
    pub qpos: Vec<VectorNf<D>>,
    pub qvel: Vec<VectorNf<D>>,
    pub qacc: Vec<VectorNf<D>>,
}

impl <D: Dim + DimName> JointSpaceTrajectory<D>
    where DefaultAllocator: Allocator<f32, D> {

    /// Generate trajectories with trapezoidal velocity profiles
    ///
    /// Generate piecewise polynomials through multiple waypoints using trapezoidal
    /// velocity profiles. It computes a trajectory through a given set of input WAYPOINTS.
    /// The function outputs positions, Q, velocities, QD, and accelerations, QDD, at the
    /// given time samples, T.
    ///
    /// # Arguments
    ///
    /// * `way_points` - A Vec of VectorN<f32, D> with len N specifying N waypoints that
    /// are each an Dx1 vector of positions.
    /// * `num_samples` - A scalar indicating the number of samples, M.
    ///
    /// # Reference
    /// [1] K. Lynch and F. Park, Modern Robotics: Mechanics, Planning, and
    /// Control. Cambridge, UK: Cambridge University Press, 2017.
    /// [2] M. Spong, S. Hutchinson, S. Vidyasagar, Robot Modeling and
    /// Control. John Wiley & Sons, Inc., 2006.
    pub fn from_trap_vel(way_points: &Vec<VectorN<f32, D>>, num_samples: usize) {

        // establish some dimensions
        let m = num_samples;
        let n = way_points[0].nrows(); // dimension of the waypoint
        let p = way_points.len();      // number of waypoints

        // initialize outputs
        let mut q   = vec![VectorNf::<D>::zeros(); m];
        let mut qd  = vec![VectorNf::<D>::zeros(); m];
        let mut qdd = vec![VectorNf::<D>::zeros(); m];

        // compute parameters, coefficients, and breaks, and store for subsequent
        // use in pp-form and to generate outputs.
        let parameter_mat = zeros_3d(n, p-1, 6);

        // Initialize the coefficient matrix to zeros. For P waypoints, there are
        // (P-1 segments) x (N dimensions) of polynomials, each with four
        // coefficients.
        let coeff_mat = zeros(3*(p-1), 3);
        let break_mat = vec![VectorNf::<D>::zeros(); 3*(p-1)+1];
        for i in 0..n {
            for p in 0..p - 1 {
                // Compute the trapezoidal profile parameters for the profile
                // segment between two waypoints. Since the user-specified
                // parameter values may be empty here, pass the indices as
                // inputs following function.
            }
        }

        println!("{:?}", parameter_mat);

        JointSpaceTrajectory { qacc: qdd, qvel: qd, qpos: q, };
    }

    pub fn compute_profile_params(i: usize, j: usize, way_points: &Vec<VectorNf<D>>)
        -> (f32, f32) {
        unimplemented!()
    }

    pub fn compute_scalar_lspb_coeffs(wp1: f32, wp2: f32, vel_seg: f32, acc_seg: f32) {
        unimplemented!()
    }
}

