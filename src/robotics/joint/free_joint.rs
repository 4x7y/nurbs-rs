use crate::math::matrix::*;
use std::f32;
use crate::robotics::{Joint, JointType};

pub struct FreeJoint {
    pub name: String,
    pub id: usize,
    pub is_valid: bool,
    pub xpos: Vector3f,
    pub quat: Vector4f,
    pub xvel: Vector3f,
}

impl Joint for FreeJoint {
    fn new() -> Self {
        return FreeJoint {
            name: String::from(""),
            id: 0,
            is_valid: true,
            xpos: Vector3f::zeros(),
            quat: Vector4f::zeros(),
            xvel: Vector3f::zeros(),
        };
    }

    fn is_valid(&self) -> bool {
        return self.is_valid;
    }

    fn get_xpos(&self) -> Vector3f {
        return self.xpos;
    }

    fn get_xvel(&self) -> Vector3f {
        return self.xvel;
    }

    fn get_ndof(&self) -> usize {
        return 6;
    }

    fn get_joint_id(&self) -> usize {
        return self.id;
    }

    fn get_name(&self) -> &String {
        return &self.name;
    }

    fn get_joint_type() -> JointType {
        return JointType::Free;
    }
}