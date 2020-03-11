// use crate::math::matrix::*;
// use std::f32;
// use crate::robotics::{Joint, JointType};
//
// pub struct RevoluteJoint {
//     pub ndof: usize,
//     pub is_valid: bool,
//     pub name: String,
//     pub joint_id: usize,
//     pub robot_id: usize,
//     pub screw: Screw,
//     pub limit: (f32, f32),
// }
//
// impl Joint for RevoluteJoint {
//     fn new() -> RevoluteJoint {
//         return RevoluteJoint {
//             ndof: 1,
//             is_valid: true,
//             name: String::from(""),
//             joint_id: 0,
//             robot_id: 0,
//             screw: Screw::zeros(),
//             limit: (f32::NEG_INFINITY, f32::INFINITY),
//         };
//     }
//
//     fn is_valid(&self) -> bool {
//         return self.is_valid;
//     }
//
//     fn get_xpos(&self) -> Vector3f {
//         return Vector3f::zeros();
//     }
//
//     fn get_xvel(&self) -> Vector3f {
//         return Vector3f::zeros();
//     }
//
//     fn get_ndof(&self) -> usize {
//         return 1;
//     }
//
//     fn get_joint_id(&self) -> usize {
//         return self.joint_id;
//     }
//
//     fn get_name(&self) -> &String {
//         return &self.name;
//     }
//
//     fn get_joint_type() -> JointType {
//         return JointType::Revolute;
//     }
// }