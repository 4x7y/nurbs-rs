use crate::math::Vector3f;

pub enum JointType {
    Free,        // global position and orientation (quat)       (ndof 7)
    Ball,        // orientation (quat) relative to parent        (ndof 4)
    Prismatic,   // sliding distance along body-fixed axis       (ndof 1)
    Revolute,    // rotation angle (rad) around body-fixed axis  (ndof 1)
    Fixed,       // fixed joint                                  (ndof 0)
}

pub trait Joint {
    fn new() -> Self;

    fn is_valid(&self) -> bool;

    fn get_xpos(&self) -> Vector3f;

    fn get_xvel(&self) -> Vector3f;

    fn get_ndof(&self) -> usize;

    fn get_joint_id(&self) -> usize;

    fn get_name(&self) -> &String;

    fn get_joint_type() -> JointType;
}