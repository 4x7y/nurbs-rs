use crate::math::*;
use na::geometry::{Translation3, UnitQuaternion};
use crate::robotics::Range;
use std::fmt;

#[derive(Copy, Debug, Clone)]
pub enum JointType {
    // sliding distance along body-fixed axis       (ndof 1)
    Prismatic{ axis: UnitVector3f, },
    // rotation angle (rad) around body-fixed axis  (ndof 1)
    Revolute { axis: UnitVector3f, },
    // fixed                                        (ndof 0)
    Fixed,
}

/// The reason of joint error
#[derive(Debug, Clone, Fail)]
pub enum JointError {
    /// Failed to set joint angle because the input is out of range or it is fixed joint
    #[fail(display = "joint: {} is out of limit: {}", joint_name, message)]
    OutOfLimitError {
        joint_name: String,                      // name of the joint
        message: String,                         // detail error message
    },
    /// Gave invalid size of vec as input
    #[fail(display = "size mismatch input = {}, required = {}", input, required)]
    SizeMismatchError {
        input: usize,                            // size of input
        required: usize,                         // required size
    },
    /// Error about mimic
    #[fail(display = "mimic error from {} to {}", from, to)]
    MimicError {
        from: String,                            // tried to copy from `from`
        to: String,                              // tried to copy to `to`
        message: String,                         // detail error message
    },
    #[fail(display = "invalid arguments {:?}", error)]
    InvalidArgumentsError { error: String },
}


/// Joint with type
#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,                            // name
    pub joint_type: JointType,                   // type of the joint
    pub screw_axis: Vector6f,                    // screw axis
    pub position: Scalar,                        // position (angle)
    pub velocity: Scalar,                        // velocity
    pub limits: Option<Range>,                   // limits
    pub origin: Isometry3f,                      // local origin transform
    pub qpos_dof: (usize, usize),                // position DoF mapping ( <= 7 )
    pub qvel_dof: (usize, usize),                // velocity DoF mapping ( <= 6 )
}


impl Joint {
    pub fn new(name: &str, joint_type: JointType) -> Joint {
        Joint {
            name: name.to_string(),
            joint_type: joint_type,
            screw_axis: Vector6f::zeros(),
            position: 0.,
            velocity: 0.,
            limits: None,
            origin: Isometry3f::identity(),
            qpos_dof: (0, 0),
            qvel_dof: (0, 0)
        }
    }

    pub fn set_joint_position(&mut self, position: Scalar) -> Result<(), JointError> {
        if let JointType::Fixed = self.joint_type {
            return Err(JointError::OutOfLimitError {
                joint_name: self.name.to_string(),
                message: "Joint is Fixed".to_owned(),
            });
        }
        if let Some(ref range) = self.limits {
            if !range.is_valid(position) {
                return Err(JointError::OutOfLimitError {
                    joint_name: self.name.to_string(),
                    message: format!(
                        "Joint is out of range: input={}, range={:?}",
                        position, range
                    ),
                });
            }
        }
        self.position = position;
        Ok(())
    }

    pub fn set_joint_position_unchecked(&mut self, position: Scalar) {
        self.position = position;
    }

    /// Returns the position (angle)
    #[inline]
    pub fn joint_position(&self) -> Option<Scalar> {
        match self.joint_type {
            JointType::Fixed => None,
            _ => Some(self.position),
        }
    }

    #[inline]
    pub fn origin(&self) -> &Isometry3f {
        &self.origin
    }

    #[inline]
    pub fn set_origin(&mut self, origin: Isometry3f) {
        self.origin = origin;
    }

    pub fn set_joint_velocity(&mut self, velocity: Scalar) -> Result<(), JointError> {
        if let JointType::Fixed = self.joint_type {
            return Err(JointError::OutOfLimitError {
                joint_name: self.name.to_string(),
                message: "Joint is Fixed".to_owned(),
            });
        }
        self.velocity = velocity;
        Ok(())
    }

    /// Returns the velocity
    #[inline]
    pub fn joint_velocity(&self) -> Option<Scalar> {
        match self.joint_type {
            JointType::Fixed => None,
            _ => Some(self.velocity),
        }
    }

    pub fn local_transform(&self) -> Isometry3f {
        let joint_transform = match self.joint_type {
            JointType::Fixed => Isometry3f::identity(),
            JointType::Revolute { axis } => Isometry3f::from_parts(
                Translation3::new(0., 0., 0.),
                UnitQuaternion::from_axis_angle(&axis, self.position),
            ),
            JointType::Prismatic { axis } => Isometry3f::from_parts(
                Translation3::from(axis.into_inner() * self.position),
                UnitQuaternion::identity(),
            ),
        };
        self.origin * joint_transform
    }

    #[inline]
    pub fn is_movable(&self) -> bool {
        match self.joint_type {
            JointType::Fixed => false,
            _ => true,
        }
    }
}

impl fmt::Display for Joint {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.name)
    }
}

// pub trait Joint {
//     fn new() -> Self;
//
//     fn is_valid(&self) -> bool;
//
//     fn get_xpos(&self) -> Vector3f;
//
//     fn get_xvel(&self) -> Vector3f;
//
//     fn get_ndof(&self) -> usize;
//
//     fn get_joint_id(&self) -> usize;
//
//     fn get_name(&self) -> &String;
//
//     fn get_joint_type() -> JointType;
// }
