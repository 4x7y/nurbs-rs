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

pub fn spatial_inertia(mass: Scalar, com: Vector3f, inertia: Matrix3f) -> Matrix6f {
    let sc = skew(com);
    let mut spatial_inertia = Matrix6f::zeros();
    spatial_inertia.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&inertia);
    spatial_inertia.fixed_slice_mut::<U3, U3>(0, 3).copy_from(&(mass * sc));
    spatial_inertia.fixed_slice_mut::<U3, U3>(3, 0).copy_from(&(mass * sc.transpose()));
    spatial_inertia.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&(mass * Matrix3f::identity()));

    return spatial_inertia;
}