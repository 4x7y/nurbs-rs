use crate::math::matrix::*;
use crate::robotics::{Link, Joint, JointType, JointPosition};
use std::fmt::Display;
use failure::_core::fmt::{Formatter, Error};
use prettytable::{Cell, Table};
use std::fmt;

#[derive(Debug, Clone)]
pub struct RigidBody {
    pub link: Link,                             // link
    pub joint: Option<Joint>,                   // joint
    pub index: usize,                           // index in the RBTree
    pub parent_index: Option<usize>,            // index of the parent in the RBTree
    pub qvel_dof_map: (usize, usize),           //
    pub qpos_dof_map: (usize, usize),           //
    pub in_tree: bool,                          //
}


impl RigidBody {
    pub fn from_link(link: Link, in_tree: bool) -> Self {
        RigidBody {
            link: link,
            joint: None,
            index: 0,
            parent_index: None,
            qvel_dof_map: (0, 0),
            qpos_dof_map: (0, 0),
            in_tree: in_tree,
        }
    }

    pub fn joint_name(&self) -> String {
        match &self.joint {
            None => "None".to_string(),
            Some(joint) => joint.name.clone(),
        }
    }

    pub fn name(&self) -> String {
        self.link.name.clone()
    }

    pub fn qpos_dof(&self) -> usize {
        match &self.joint {
            None => { 0 },
            Some(joint) => { joint.qpos_dof() },
        }
    }

    pub fn qvel_dof(&self) -> usize {
        match &self.joint {
            None => { 0 },
            Some(joint) => { joint.qpos_dof() },
        }
    }

    pub fn get_qpos_from_vec(&self, qpos: &VectorDf, start: usize) -> JointPosition {
        match &self.joint {
            None => JointPosition::Fixed,
            Some(joint) => joint.get_qpos_from_vec(qpos, start)
        }
    }

    pub fn joint_type(&self) -> Option<JointType> {
        match &self.joint {
            None => None,
            Some(joint) => Some(joint.joint_type.clone()),
        }
    }

    pub fn joint_type_name(&self) -> String {
        match &self.joint {
            None => "None".to_string(),
            Some(joint) => joint.joint_type.to_string(),
        }
    }

    pub fn tform_body2parent(&self, qpos: JointPosition) -> Matrix4f {
        match &self.joint {
            None => Matrix4f::identity(),
            Some(joint) => joint.tform_body2parent(qpos),
        }
    }

    pub fn qvel_dof_map(&self) -> (usize, usize) {
        self.qvel_dof_map
    }

    pub fn qpos_dof_map(&self) -> (usize, usize) {
        self.qpos_dof_map
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
        table.add_row(row!["Body Name", "Joint", "Geometry Type", "Inertial"]);
        table.add_row(row![
            Cell::new(&self.link.name),
            Cell::new(&self.joint_name()),
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