use crate::math::matrix::*;

pub struct RigidBody {
    pub mass: f32,                              // mass
    pub mm: Matrix3f,                           // inertia tensor
    pub mm_vec: Vector6f,                       // inertia vector
    pub com:  Vector3f,                         // center of mass
    pub name: String,                           // name
}

pub struct RigidBodyState {
    pub xpos: Vector3f,                         // position in the world frame
    pub xvel: Vector3f,                         // velocity in the world frame
    pub xacc: Vector3f,                         // acceleration in the world frame

    pub quat: Vector4f,                         // quaternion relative to world frame
    pub omeg: Vector3f,                         // angular velocity in the world frame
    pub alph: Vector3f,                         // angular acceleration in the world frame
}