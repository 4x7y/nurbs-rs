use crate::robotics::*;
use crate::math::*;

/// Computes the mass matrix of an open chain robot based on the given
/// configuration. It returns the numerical inertia matrix `M(qpos)` of
/// an `n`-joint serial chain at the given configuration `qpos`.
///
/// This function calls `rne` for `nv` times, each time passing a
/// `qacc` vector with a single element equal to one and all other
/// inputs set to zero.
pub fn mass_matrix(model: &RobotModel, state: &RobotState) -> MatrixDDf {
    let n = model.nv;
    let mut mass_matrix = MatrixDDf::zeros(n, n);
    let qvel_zero = VectorDf::zeros(n);
    let xfrc_zero = Vector6f::zeros();
    let gravity_zero = Vector3f::zeros();
    for i in 0..n {
        let mut qacc_one = VectorDf::zeros(n);
        qacc_one[i] = 1.;

        mass_matrix.slice_mut((0, i), (n, 1)).copy_from(
            &rne(model, &state.qpos, &qvel_zero, &qacc_one, &gravity_zero, &xfrc_zero)
        );
    }

    return mass_matrix;
}

/// Computes the Coriolis and centripetal terms in the inverse dynamics of
/// an open chain robot. It returns the vector `C(qpos,qvel)` of Coriolis
/// and centripetal terms for a given `qpos` and `qvel`.
///
/// This function calls InverseDynamics with `g = 0`, `fext = 0`, and
/// `qacc = 0`.
pub fn velocity_product(robot: &RobotModel, state: &RobotState) -> VectorDf {
    let qacc_zero = VectorDf::zeros(robot.nv);
    let xfrc_zero = Vector6f::zeros();
    let gravity_zero = Vector3f::zeros();
    return rne(robot, &state.qpos, &state.qvel, &qacc_zero, &gravity_zero, &xfrc_zero);
}

/// Computes the joint forces/torques an open chain robot requires to
/// overcome gravity at its configuration
///
/// This function calls `rne` with `fext = 0`, `qvel = 0`, and `qacc = 0`.
pub fn gravity_forces(robot: &RobotModel, state: &RobotState) -> VectorDf {
    let qvel = VectorDf::zeros(robot.nv);
    let qacc = VectorDf::zeros(robot.nv);
    let xfrc = Vector6f::zeros();
    return rne(robot, &state.qpos, &qvel, &qacc, &robot.gravity, &xfrc);
}

/// Computes the joint forces/torques an open chain robot requires only to
/// create the end-effector force `fext`
pub fn end_effector_forces(robot: &RobotModel, state: &RobotState, fext: &Vector6f) -> VectorDf {
    let qvel = VectorDf::zeros(robot.nv);
    let qacc = VectorDf::zeros(robot.nv);
    let gravity_zero = Vector3f::zeros();
    return rne(robot, &state.qpos, &qvel, &qacc, &gravity_zero, &fext);
}

/// Computes forward dynamics in the space frame for an open chain robot
pub fn forward_dynamics(robot: &RobotModel, state: &RobotState,
                        tau: &VectorDf, xfrc_tip: &Vector6f) -> VectorDf {
    let mass_matrix = mass_matrix(robot, state);
    // mass matrix must be invertible, directly call unwrap here
    let mass_matrix_inv = mass_matrix.try_inverse().unwrap();
    let mut qacc = VectorDf::zeros(robot.nv);
    qacc += mass_matrix_inv * tau;
    qacc -= velocity_product(robot, state);
    qacc -= gravity_forces(robot, state, );
    qacc -= end_effector_forces(robot, state, xfrc_tip);

    return qacc;
}