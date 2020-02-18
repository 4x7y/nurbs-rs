use crate::math::*;
use crate::robotics::*;

/// Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm.
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
/// the joint-space equation of motion, `tau=H(q)qdd+C(d,qd,f_ext)`, where `q,`
/// `qd and qdd` are the joint position, velocity and acceleration vectors, `H`
/// is the joint-space inertia matrix, `C` is the vector of gravity,
/// external-force and velocity-product terms, and tau is the joint force
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
pub fn crba(robot: &Robot, q: &VectorDf) -> MatrixDDf {

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
pub fn rne(robot: &Robot) {
    let gravity = robot.gravity.clone();
    let zhat = Vector3f::new(0., 0., 1.);

    for k in 0..robot.nv {

    }

}