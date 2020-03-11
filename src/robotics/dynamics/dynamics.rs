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

