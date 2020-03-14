use crate::math::matrix::*;

trait MotionModel {
    fn joint_space_motion();                    // tmp space motion
    fn task_space_motion();                     // task space motion
}

pub(crate) trait Kinematics {
    // get transformation between body frame
    fn get_transform(&self, qpos: VectorDf, body_from: String, body_to: String) -> Matrix4f;

    // get random configuration
    fn random_configuration(&self) -> VectorDf;

    // get home configuration
    fn home_configuration(&self) -> VectorDf;

    // get geometric jacobian matrix
    fn geometric_jacobian(&self) -> MatrixDDf;
}


/// Determines whether a scalar is small enough to be treated as zero
pub fn near_zero(value: Scalar) ->bool {
    return value.abs() < 1.0e-6;
}


/// Normalizes a vector
pub fn normalize(vec: Vector3f) -> Vector3f {
    return vec / vec.norm();
}

/// Converts a homogeneous transformation matrix into a rotation matrix
/// and position vector
pub fn trans_to_rp(trans: Matrix4f) -> (Matrix3f, Vector3f) {
    let rotm = trans.fixed_slice::<U3, U3>(0, 0);
    let tvec = trans.fixed_slice::<U3, U1>(0, 3);
    return (Matrix3f::from(rotm), Vector3f::from(tvec));
}

/// Converts a 3-vector to an so(3) representation
pub fn vec_to_so3(omeg: Vector3f) -> Matrix3f {
    Matrix3f::new(
        0., -omeg[2], omeg[1],
        omeg[2], 0., -omeg[0],
        -omeg[1], omeg[0], 0.
    )
}

/// Return skew symmetric matrix of a 3-Vector
pub fn skew(omeg: Vector3f) -> Matrix3f {
    vec_to_so3(omeg)
}

/// Computes the adjoint representation of a homogeneous transformation
/// matrix
///
/// # Arguments
///
/// - `trans`: A 4x4 homogeneous transformation matrix
///
/// # Return
///
/// - The 6x6 adjoint representation Ad T of T
pub fn adjoint(trans: Matrix4f) -> Matrix6f {
    let rp = trans_to_rp(trans);
    let rotm = rp.0;
    let tvec = rp.1;
    let tmp = vec_to_so3(tvec) * rotm;

    let mut adt = Matrix6f::zeros();
    adt.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&rotm);
    adt.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&tmp);
    adt.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&rotm);

    return adt;
}

/// Inverts a homogeneous transformation matrix
///
/// # Arguments
///
/// `trans`: A homogeneous transformation matrix
pub fn trans_inv(trans: Matrix4f) -> Matrix4f {
    let rp = trans_to_rp(trans);
    let rotm = rp.0;
    let tvec = rp.1;
    let rotm_t = rotm.transpose();

    let mut trans_inv = Matrix4f::zeros();
    trans_inv.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&rotm_t);
    let tmp = - rotm_t * tvec;
    trans_inv.fixed_slice_mut::<U3, U1>(0, 3).copy_from(&tmp);
    trans_inv[(3, 3)] = 1.;

    return trans_inv;
}

/// Converts a spatial velocity vector into a 4x4 matrix in se3
pub fn vec_to_se3(velc: Vector6f) -> Matrix4f {
    let mut se3 = Matrix4f::zeros();
    let omeg = Vector3f::new(velc[0], velc[1], velc[2]);
    let tran = Vector3f::new(velc[3], velc[4], velc[5]);
    se3.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&vec_to_so3(omeg));
    se3.fixed_slice_mut::<U3, U1>(0, 3).copy_from(&tran);

    return se3;
}

/// Converts an so(3) representation to a 3-vector
pub fn so3_to_vec(so3mat: Matrix3f) -> Vector3f {
    Vector3f::new(
        so3mat[(2, 1)],
        so3mat[(0, 2)],
        so3mat[(1, 0)])
}

/// Converts a 3-vector of exponential coordinates for rotation into
/// axis-angle form
///
/// # Arguments
/// - `expc3`: A 3-vector of exponential coordinates for rotation
pub fn axis_angle(expc3: Vector3f) -> (Vector3f, Scalar) {
    let ang = expc3.norm();
    let vec = normalize(expc3);
    return (vec, ang);
}

/// Calculate the 6x6 matrix [adV] of the given 6-vector
pub fn ad(v: Vector6f) -> Matrix6f {
    let omegmat = vec_to_so3(Vector3f::new(v[0], v[1], v[2]));
    let tmp = vec_to_so3(Vector3f::new(v[3], v[4], v[5]));
    let mut adv = Matrix6f::zeros();
    adv.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&omegmat);
    adv.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&tmp);
    adv.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&omegmat);

    return adv;
}


/// Rodrigues's rotation formula
pub fn rodrigues(theta: Scalar, omegmat: Matrix3f) -> Matrix3f {
    return Matrix3f::identity()
        + theta.sin() * omegmat
        + (1. - theta.cos()) * omegmat * omegmat;
}

/// Computes the matrix exponential of a matrix in so(3)
pub fn matrix_exp3(so3mat: Matrix3f) -> Matrix3f {
    let omeg_theta = so3_to_vec(so3mat);
    return if near_zero(omeg_theta.norm()) {
        Matrix3f::identity()
    } else {
        let theta = axis_angle(omeg_theta).1;
        let omegmat = so3mat / theta;
        rodrigues(theta, omegmat)
    }
}

/// Computes the matrix exponential of an se3 representation of
/// exponential coordinates
pub fn matrix_exp6(se3mat: Matrix4f) -> Matrix4f {

    let so3 = Matrix3f::from(se3mat.fixed_slice::<U3, U3>(0, 0));
    let omeg_theta = so3_to_vec(so3);
    let mut tform = Matrix4f::zeros();

    if near_zero(omeg_theta.norm()) {
        tform.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&Matrix3f::identity());
        tform.fixed_slice_mut::<U3, U1>(0, 3).copy_from(&se3mat.fixed_slice::<U3, U1>(0, 3));
        tform[(3, 3)] = 1.;
    } else {
        let theta = axis_angle(omeg_theta).1;
        let omegmat = se3mat.fixed_slice::<U3, U3>(0, 0) / theta;
        let tmp = matrix_exp3(Matrix3f::from(se3mat.fixed_slice::<U3, U3>(0, 0)));
        tform.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&tmp);
        let tmp = Matrix3f::identity() * theta
                + (1. - theta.cos()) * omegmat
                + (theta - theta.sin()) * omegmat * omegmat;
        let tmp = tmp * se3mat.fixed_slice::<U3, U1>(0, 3) / theta;
        tform.fixed_slice_mut::<U3, U1>(0, 3).copy_from(&tmp);
        tform[(3, 3)] = 1.;
    }

    return tform;
}


/// Convert homogeneous transformation (4x4) to its adjoint spatial
/// transformation (6x6).
///
/// This function should have the same implementation as `adjoint`
pub fn tform_to_spatial_xform(tform: Matrix4f) -> Matrix6f {
    let rotm = tform.fixed_slice::<U3, U3>(0, 0);
    let tvec = tform.fixed_slice::<U3, U1>(0, 3);
    let tvec_skew = skew(Vector3f::from(tvec));

    let mut xform = Matrix6f::zeros();
    xform.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&rotm);
    xform.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&(tvec_skew * rotm));
    xform.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&rotm);

    return xform;
}