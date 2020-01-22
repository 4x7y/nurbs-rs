use crate::math::vector::{Vector3f, Screw};

pub enum JointType {
    Free,        // global position and orientation (quat)       (7)
    BALL,        // orientation (quat) relative to parent        (4)
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

    fn get_id(&self) -> usize;

    fn get_name(&self) -> &String;
}

pub struct RevoluteJoint {
    pub ndof: usize,
    pub is_valid: bool,
    pub name: String,
    pub id: usize,
    pub screw: Screw
}

impl Joint for RevoluteJoint {
    fn new() -> RevoluteJoint {
        return RevoluteJoint { 
            ndof: 1, 
            is_valid: true,
            name: String::from(""),
            id: 0,
            screw: Screw::zeros(),
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

    fn get_id(&self) -> usize {
        return self.id;
    }

    fn get_name(&self) -> &String { 
        return &self.name;
    }
}