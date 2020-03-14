use crate::math::matrix::*;
use crate::robotics::*;

pub(crate) trait Dynamics {
    fn center_of_mass();                        // get center of mass
    fn mass_matrix();                           // get tmp space mass matrix
    fn forward_dynamics();                      // get tmp acceleration given tmp torques and states
    fn inverse_dynamics();                      // get required tmp torques for given motion
    fn velocity_product();                      // get tmp torques that cancel velocity-induced forces
    fn gravity_torque();                        // get tmp torques that compensate gravity
}

/// Form the 6x6 spatial inertia matrix
///
/// # Arguments
///
/// - `mass`: rigid body mass
/// - `bvec_com`: center of mass of the rigid body relative to body frame
/// - `bmat_inertia`: 3x3 inertia tensor of the rigid body relative to body frame
///
/// # Note
///  The expression here is different to Featherstone's book Eq. (2.63) since the
///  inertia tensor is evaluated at the body frame instead of CoM.
pub fn spatial_inertia(mass: Scalar, bvec_com: Vector3f, bmat_inertia: Matrix3f) -> Matrix6f {
    let sc = skew(bvec_com);
    let mut spatial_inertia = Matrix6f::zeros();
    spatial_inertia.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&bmat_inertia);
    spatial_inertia.fixed_slice_mut::<U3, U3>(0, 3).copy_from(&(mass * sc));
    spatial_inertia.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&(mass * sc.transpose()));
    spatial_inertia.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&(mass * Matrix3f::identity()));

    return spatial_inertia;
}

/// Transform inertia matrix from the origin of body frame {B} to center of mass in {B}
///
/// # Arguments
///
/// - `mass`: rigid body mass
/// - `bvec_com`: center of mass of the rigid body relative to body frame
/// - `bmat_inertia`: 3x3 inertia tensor of the rigid body relative to body frame
pub fn inertia_body2com(mass: Scalar, bvec_com: Vector3f, bmat_inertia: Matrix3f) -> Matrix3f {
    return bmat_inertia - mass * skew(bvec_com) * skew(bvec_com).transpose();
}

/// Transform inertia matrix from the origin of body frame {B} to center of mass in {B}
///
/// # Arguments
///
/// - `mass`: rigid body mass
/// - `bvec_com`: center of mass of the rigid body relative to body frame
/// - `cmat_inertia`: 3x3 inertia tensor of the rigid body relative to a
///                   frame located at CoM with axis aligned to body frame
pub fn inertia_com2body(mass: Scalar, bvec_com: Vector3f, cmat_inertia: Matrix3f) -> Matrix3f {
    return cmat_inertia + mass * skew(bvec_com) * skew(bvec_com).transpose();
}