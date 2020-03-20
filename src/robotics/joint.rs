use crate::math::*;
use na::geometry::{Translation3, UnitQuaternion};
use crate::robotics::Range;
use std::fmt;
use failure::_core::fmt::{Formatter, Error};
use crate::utils::*;
use prettytable::{Cell, Row, Table};

#[derive(Copy, Debug, Clone)]
pub enum JointType {
    // sliding distance along body-fixed axis       (ndof 1)
    Prismatic{ axis: UnitVector3f, },
    // rotation angle (rad) around body-fixed axis  (ndof 1)
    Revolute { axis: UnitVector3f, },
    // fixed                                        (ndof 0)
    Fixed,
    // temp
    Unspecified,
}

impl fmt::Display for JointType {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            JointType::Prismatic { axis } => {
                write!(f, "Prismatic ({:.2}, {:.2}, {:.2})", axis[0], axis[1], axis[2])
            },
            JointType::Revolute { axis } => {
                write!(f, "Revolute ({:.2}, {:.2}, {:.2})", axis[0], axis[1], axis[2])
            },
            JointType::Fixed => {
                write!(f, "Fixed")},
            JointType::Unspecified => {
                write!(f, "Unspecified")
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct JointDynamics {
    pub damping: Scalar,
    pub friction: Scalar,
    pub effort_limit: Scalar,
}

#[derive(Debug, Clone)]
pub struct JointSafetyController {
    pub soft_lower_limit: Scalar,
    pub soft_upper_limit: Scalar,
    pub k_position: Scalar,
    pub k_velocity: Scalar,
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
    pub name: String,                             // name
    pub joint_type: JointType,                    // type of the joint
    pub screw_axis: Vector6f,                     // screw axis
    pub qpos_home: Scalar,                        // home configuration of the joint
    pub qpos_limit: Option<Range>,                // joint position limits {None, (min, max)}
    pub qvel_limit: Option<Range>,                // joint velocity limits {None, (  0, max)}
    pub dynamics: Option<JointDynamics>,          // joint dynamics coefficients (damping, friction)
    pub safe_ctrl: Option<JointSafetyController>, // joint safety controller
    pub tform_jnt2parent: Matrix4f,               // fixed transform from joint to parent frame
    pub tform_child2jnt: Matrix4f,                // fixed transform from child to joint frame
}


impl Joint {
    pub fn new<S>(name: S, joint_type: JointType) -> Joint
        where S: Into<String> {
        Joint {
            name: name.into(),
            joint_type: joint_type,
            screw_axis: Vector6f::zeros(),
            qpos_home: 0.0,
            qpos_limit: None,
            qvel_limit: None,
            dynamics: None,
            safe_ctrl: None,
            tform_jnt2parent: Matrix4f::identity(),
            tform_child2jnt: Matrix4f::identity(),
        }
    }

    // pub fn set_joint_position(&mut self, position: Scalar) -> Result<(), JointError> {
    //     if let JointType::Fixed = self.joint_type {
    //         return Err(JointError::OutOfLimitError {
    //             joint_name: self.name.to_string(),
    //             message: "Joint is Fixed".to_owned(),
    //         });
    //     }
    //     if let Some(ref range) = self.qpos_limit {
    //         if !range.is_valid(position) {
    //             return Err(JointError::OutOfLimitError {
    //                 joint_name: self.name.to_string(),
    //                 message: format!(
    //                     "Joint is out of range: input={}, range={:?}",
    //                     position, range
    //                 ),
    //             });
    //         }
    //     }
    //     self.position = position;
    //     Ok(())
    // }
    //
    // pub fn set_joint_position_unchecked(&mut self, position: Scalar) {
    //     self.position = position;
    // }

    // /// Returns the position (angle)
    // #[inline]
    // pub fn joint_position(&self) -> Option<Scalar> {
    //     match self.joint_type {
    //         JointType::Fixed => None,
    //         _ => Some(self.position),
    //     }
    // }
    //
    // #[inline]
    // pub fn origin(&self) -> &Isometry3f {
    //     &self.tform_jnt2parent
    // }

    #[inline]
    pub fn set_fixed_tform(&mut self, tform: Matrix4f) {
        self.tform_jnt2parent = tform;
    }

    // pub fn set_joint_velocity(&mut self, velocity: Scalar) -> Result<(), JointError> {
    //     if let JointType::Fixed = self.joint_type {
    //         return Err(JointError::OutOfLimitError {
    //             joint_name: self.name.to_string(),
    //             message: "Joint is Fixed".to_owned(),
    //         });
    //     }
    //     self.velocity = velocity;
    //     Ok(())
    // }
    //
    // /// Returns the velocity
    // #[inline]
    // pub fn joint_velocity(&self) -> Option<Scalar> {
    //     match self.joint_type {
    //         JointType::Fixed => None,
    //         _ => Some(self.velocity),
    //     }
    // }

    // pub fn local_transform(&self) -> Isometry3f {
    //     let joint_transform = match self.joint_type {
    //         JointType::Fixed => Isometry3f::identity(),
    //         JointType::Revolute { axis } => Isometry3f::from_parts(
    //             Translation3::new(0., 0., 0.),
    //             UnitQuaternion::from_axis_angle(&axis, self.position),
    //         ),
    //         JointType::Prismatic { axis } => Isometry3f::from_parts(
    //             Translation3::from(axis.into_inner() * self.position),
    //             UnitQuaternion::identity(),
    //         ),
    //     };
    //     self.tform_jnt2parent * joint_transform
    // }

    #[inline]
    pub fn is_movable(&self) -> bool {
        match self.joint_type {
            JointType::Fixed => false,
            _ => true,
        }
    }

    pub fn show_details(&self) {
        // name: name.to_string(),
        // joint_type: joint_type,
        // screw_axis: Vector6f::zeros(),
        // position: 0.,
        // velocity: 0.,
        // limits: None,
        // origin: Isometry3f::identity(),
        // qpos_dof: (0, 0),
        // qvel_dof: (0, 0)

        let mut table = Table::new();
        table.add_row(row!["name", "type", "screw_axis", "limits", "ypr"]);
        let rpy = rotm2eul(Matrix3f::from(self.tform_jnt2parent.fixed_slice::<U3, U3>(0, 0)),
                           EulerAngleOrder::ZYX);
        table.add_row(row![
            Cell::new(format!("{}", self.name).as_ref()),
            Cell::new(format!("{}", self.joint_type).as_ref()),
            Cell::new(format!("({}, {}, {}, {}, {}, {})",
                self.screw_axis[0], self.screw_axis[1], self.screw_axis[2],
                self.screw_axis[3], self.screw_axis[4], self.screw_axis[5]).as_ref()),
            Cell::new(match self.qpos_limit {
                Some(range) => format!("{}, {}", range.min, range.max),
                None => "None".to_string()
            }.as_ref()),
            Cell::new(format!("{}", rpy).as_ref()),
            // Cell::new(self.origin),
            // Cell::new(self.position),
            // Cell::new(self.velocity),
        ]);
        table.printstd();
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
