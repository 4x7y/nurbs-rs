use crate::math::*;
use crate::robotics::*;
use std::intrinsics::transmute;

/// Computes the tmp space inertia matrix by using the Composite Rigid Body Algorithm.
///
/// # Arguments
///
/// * `robot` - robot
///
/// # Reference
/// Composite Rigid Body Algorithm
///
/// HandC  Calculate coefficients of equation of motion.
/// `[H,C] = HandC(model,q,qd,f_ext,grav_accn)` calculates the coefficients of
/// the tmp-space equation of motion, `tau=H(q)qdd+C(d,qd,f_ext)`, where `q,`
/// `qd and qdd` are the tmp position, velocity and acceleration vectors, `H`
/// is the tmp-space inertia matrix, `C` is the vector of gravity,
/// external-force and velocity-product terms, and tau is the tmp force
/// vector.
///
/// Algorithm: recursive Newton-Euler for `C`, and Composite-Rigid-Body for `H`.
/// f_ext is a cell array specifying external forces acting on the bodies.
/// If `f_ext == {}` then there are no external forces; otherwise, `f_ext{i}` is
/// a spatial force vector giving the force
/// acting on body i, expressed in body i coordinates.  Empty cells in f_ext
/// are interpreted as zero forces.  grav_accn is a 3D vector expressing the
/// linear acceleration due to gravity.  The arguments f_ext and grav_accn
/// are optional, and default to the values {} and `[0,0,-9.81]`, respectively,
/// if omitted.
pub fn crba(robot: &RobotModel, q: &VectorDf) -> MatrixDDf {

    let mm = MatrixDDf::zeros(robot.nv, robot.nv);
    let xup = Vec::<Matrix3f>::new();

    let mut mm_rec: Vec<Matrix3f> = robot.ximm_all(q);
    for i in (0..robot.nbody).rev() {
        match robot.parent[i] {
            None => {},
            Some(indx_parent) => {
                let temp = xup[i].tr_mul(&mm_rec[indx_parent]) * &xup[i];
                mm_rec[indx_parent] += temp;
            },
        }
    }

    return mm;
}

/// Compute inverse dynamics by using the Recursive Newton Euler Algorithm.
pub fn rne(model: &RobotModel,
           qpos: &VectorDf, qvel: &VectorDf, qacc: &VectorDf,
           gravity: &Vector3f, fext: &Vector6f) -> VectorDf {
    let screw           = &model.screw;
    let tform_to_prev   = &model.tform_to_prev;
    let spatial_inertia = &model.spatial_inertia;

    let n  = model.nv;
    let mut tform = Matrix4f::identity();
    let mut screw_axis = vec![Vector6f::zeros(); n];
    let mut ad_tform = vec![Matrix6f::zeros(); n+1];
    let mut svel = vec![Vector6f::zeros(); n+1];
    let mut sacc = vec![Vector6f::zeros(); n+1];
    sacc[0].fixed_slice_mut::<U3, U1>(3, 0).copy_from(&-gravity);
    ad_tform[n] = adjoint(trans_inv(tform_to_prev[n]));

    let mut xfrc = fext.clone();
    let mut tau = VectorDf::zeros(n);

    // Forward Iteration
    // The positions, velocities, and accelerations of each link are propagated from
    // the base to the tip.
    for i in 0..n {
        tform = tform * tform_to_prev[i];
        let tform_curr = trans_inv(tform);
        screw_axis[i] = adjoint(tform_curr) * screw[i];
        ad_tform[i]   = adjoint(
            matrix_exp6(vec_to_se3(- screw_axis[i] * qpos[i]))
                * trans_inv(tform_to_prev[i]));
        svel[i+1] = ad_tform[i] * svel[i] + screw_axis[i] * qvel[i];
        sacc[i+1] = ad_tform[i] * sacc[i] + screw_axis[i] * qacc[i]
            + ad(svel[i+1]) * screw_axis[i] * qvel[i];
    }

    // Backward Iteration
    // The forces and moments experienced by each link are propagated from tip to the
    // base.
    for i in (0..n).rev() {
        xfrc = ad_tform[i+1].transpose() * xfrc;
        xfrc += spatial_inertia[i] * sacc[i+1];
        xfrc -= ad(svel[i+1]).transpose() * spatial_inertia[i] * svel[i+1];
        tau[i] = xfrc.dot(&screw_axis[i]);
    }

    return tau;
}

/// Compute inverse dynamics by using the Recursive Newton Euler Algorithm.
pub fn inverse_dynamics(model: &RobotModel, state: &RobotState, fext: &Vector6f) -> VectorDf {
    return rne(model, &state.qpos, &state.qvel, &state.qacc, &model.gravity, fext);
}


