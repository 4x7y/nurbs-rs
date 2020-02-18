use crate::math::matrix::*;

trait MotionModel {
    fn joint_space_motion();                    // joint space motion
    fn task_space_motion();                     // task space motion
}

pub(crate) trait Kinematics {
    // get transformation between body frame
    fn get_transform(&self, qpos: VectorDf, body_from: String, body_to: String) -> Matrix4f;

    // get random configuration
    fn random_configuration(&self) -> Vec<f32>;

    // get home configuration
    fn home_configuration(&self) -> Vec<f32>;

    // get geometric jacobian matrix
    fn geometric_jacobian(&self) -> MatrixDDf;
}