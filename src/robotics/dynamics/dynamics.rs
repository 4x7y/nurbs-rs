use crate::math::matrix::*;
use crate::robotics::robot::Robot;

pub(crate) trait Dynamics {
    fn center_of_mass();                        // get center of mass
    fn mass_matrix();                           // get joint space mass matrix
    fn forward_dynamics();                      // get joint acceleration given joint torques and states
    fn inverse_dynamics();                      // get required joint torques for given motion
    fn velocity_product();                      // get joint torques that cancel velocity-induced forces
    fn gravity_torque();                        // get joint torques that compensate gravity
}

