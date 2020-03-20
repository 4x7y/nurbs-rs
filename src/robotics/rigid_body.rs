use crate::math::matrix::*;
use crate::robotics::{Link, Joint, JointType};
use std::fmt::Display;
use failure::_core::fmt::{Formatter, Error};
use prettytable::{Cell, Table};
use std::fmt;

#[derive(Debug, Clone)]
pub struct RigidBody {
    pub link: Link,                             // link
    pub joint: Joint,                           // joint
}


impl RigidBody {
    pub fn from_link(link: Link) -> Self {
        RigidBody {
            link: link,
            joint: Joint::new("", JointType::Fixed),
        }
    }
}

impl Display for RigidBody {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut table = Table::new();
        let mut col = "".to_string();
        for collision in self.link.collisions.iter() {
            col += &collision.name;
            col += " ";
            col += &collision.geometry.to_string();
            col += "\n";
        }
        table.add_row(row!["Body Name", "Geometry Type", "Inertial", "limits", "ypr"]);
        table.add_row(row![
            Cell::new(&self.link.name),
            Cell::new(&col),
            Cell::new(&self.link.inertial.to_string()),
        ]);

        write!(f, "{}", table.to_string())
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