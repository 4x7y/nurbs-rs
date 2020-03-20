use crate::math::matrix::*;
use crate::robotics::{Link, Joint};
use crate::robotics::JointType::Unspecified;

#[derive(Debug, Clone)]
pub struct RigidBody {
    pub link: Link,                             // link
    pub joint: Joint,                           // joint
}


impl RigidBody {
    pub fn from_link(link: Link) -> Self {
        RigidBody {
            link: link,
            joint: Joint::new("", Unspecified),
        }
    }
}


#[derive(Debug, Clone)]
pub struct RigidBodyState {
    pub xpos: Vector3f,                         // position in the world frame
    pub xvel: Vector3f,                         // velocity in the world frame
    pub xacc: Vector3f,                         // acceleration in the world frame

    pub quat: Vector4f,                         // quaternion relative to world frame
    pub omeg: Vector3f,                         // angular velocity in the world frame
    pub alph: Vector3f,                         // angular acceleration in the world frame
}