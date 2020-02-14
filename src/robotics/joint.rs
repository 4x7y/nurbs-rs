use crate::math::matrix::*;
use std::f32;

pub enum JointType {
    Free,        // global position and orientation (quat)       (7)
    Ball,        // orientation (quat) relative to parent        (4)
    Prismatic,   // sliding distance along body-fixed axis       (1)
    Revolute,    // rotation angle (rad) around body-fixed axis  (1)
    Fixed,       // fixed joint                                  (0)
}

pub trait Joint {
    fn new() -> Self;

    fn is_valid(&self) -> bool;

    fn get_xpos(&self) -> Vector3f;
    
    fn get_xvel(&self) -> Vector3f;

    fn get_ndof() -> usize;

    fn get_joint_id(&self) -> usize;

    fn get_name(&self) -> &String;

    fn get_joint_type() -> JointType;
}

pub struct RevoluteJoint {
    pub ndof: usize,
    pub is_valid: bool,
    pub name: String,
    pub joint_id: usize,
    pub robot_id: usize,
    pub screw: Screw,
    pub limit: (f32, f32),
}

impl Joint for RevoluteJoint {
    fn new() -> RevoluteJoint {
        return RevoluteJoint { 
            ndof: 1, 
            is_valid: true,
            name: String::from(""),
            joint_id: 0,
            robot_id: 0,
            screw: Screw::zeros(),
            limit: (f32::NEG_INFINITY, f32::INFINITY),
        };
    }

    fn is_valid(&self) -> bool {
        return self.is_valid;
    }

    fn get_xpos(&self) -> Vector3f {
        return Vector3f::zeros();
    }
    
    fn get_xvel(&self) -> Vector3f {
        return Vector3f::zeros();
    }

    fn get_ndof() -> usize {
        return 1;
    }

    fn get_joint_id(&self) -> usize {
        return self.joint_id;
    }

    fn get_name(&self) -> &String { 
        return &self.name;
    }

    fn get_joint_type() -> JointType {
        return JointType::Revolute;
    }
}

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

    fn get_ndof() -> usize {
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