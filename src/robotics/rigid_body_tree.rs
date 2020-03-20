use std::path::Path;
use urdf_rs::Robot;
use std::collections::HashMap;
use crate::robotics::{JointBuilder, Link, Joint, RigidBody, JointType};
use petgraph::Graph;
use petgraph::graph::{NodeIndex, EdgeIndex};
use std::fmt;
use petgraph::dot::{Dot, Config};
use log::{info, error};
use crate::utils;
use crate::math::*;
use crate::simulation::sim_model::RenderableObject;
use std::cell::RefCell;
use std::rc::Rc;
use prettytable::{Cell, Row, Table, format};


#[derive(Clone, Debug)]
pub struct RigidBodyTree {
    pub name: String,                                   // name
    dof: usize,                                         // degree of freedom
    num_fixed_body: usize,                              // number of bodies with fixed joint
    num_non_fixed_body: usize,                          // number of bodies with non fixed joint
    base: Option<Rc<RefCell<RigidBody>>>,               // root of the tree
    bodies: HashMap<String, Rc<RefCell<RigidBody>>>,    // rigid bodies
    parent: HashMap<String, String>,                    // name of the parent rigid body
    children: HashMap<String, Vec<String>>,             // name of the child rigid bodies
    joint: HashMap<String, Rc<RefCell<RigidBody>>>,     // mapping from joint name to rigid body
    body_id2ptr: Vec<Rc<RefCell<RigidBody>>>,           // mapping from id to rigid body
}

impl RigidBodyTree {

    /// Import rigid body tree from URDF file
    pub fn from_urdf_file<P>(path: P) -> Result<Self, urdf_rs::UrdfError>
    where P: AsRef<Path> {
        Ok(urdf_rs::read_file(path)?.into())
    }

    /// Create an empty RigidBodyTree model
    pub fn new(name: &String) -> Self {
        Self {
            name: name.to_string(),
            dof: 0,
            num_fixed_body: 0,
            num_non_fixed_body: 0,
            base: None,
            bodies: HashMap::new(),
            parent: HashMap::new(),
            children: HashMap::new(),
            joint: HashMap::new(),
            body_id2ptr: Vec::new(),
        }
    }

    /// Add body to RigidBodyTree
    pub fn add_link(&mut self, link: &Link, joint: &Joint, parent: &String) {
        unimplemented!();
    }

    /// Get body from RigidBodyTree
    pub fn get_body(&self, name: &str) -> RigidBody {
        if let Some(body) = self.bodies.get(name) {
            let body = body.borrow();
            return body.clone();
        } else {
            error!("link '{}' not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }

    /// Get joint from RigidBodyTree by name
    pub fn get_joint(&self, name: &str) -> Joint {
        if let Some(body) = self.joint.get(name) {
            let body = body.borrow();
            return body.joint.clone();
        } else {
            error!("link '{}' not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }

    /// Add a subtree to RigidBodyTree
    fn add_rigid_body_subtree(&mut self) {
        unimplemented!()
    }

    /// Number of bodies
    pub fn num_body(&self) -> usize {
        return self.bodies.len();
    }

    /// Number of joints
    pub fn num_joint(&self) -> usize {
        return self.joint.len();
    }

    /// Number of DoFs
    pub fn num_dof(&self) -> usize {
        return self.dof;
    }

    /// Compute the mass matrix, `M`, of the robot in the configuration `q`
    pub fn mass_matrix(&self, qpos: &VectorDf) -> MatrixDDf {
        // let nb = self.num_link();
        // let mut crb_inertia = vec![Matrix6f::zeros(); nb]; // composite-rigid-body inertia
        // let mut xforms      = vec![Matrix4f::zeros(); nb]; // spatial transform from parent of body i to body i
        // let nv = self.num_joint();
        // let mass_matrix = MatrixDDf::zeros(nv, nv);
        // let lambda_ = vec![0.; nb];
        // let lambda  = vec![0.; nv];

        // preparation
        unimplemented!();

        // return mass_matrix;
    }


    pub fn renderable_objects(&self) -> Vec<RenderableObject> {
        unimplemented!();
    }
}

impl From<urdf_rs::Robot> for RigidBodyTree {
    fn from(robot: Robot) -> Self {
        Self::from(&robot)
    }
}

impl<'a> From<&'a urdf_rs::Robot> for RigidBodyTree {

    fn from(robot: &urdf_rs::Robot) -> Self {
        let mut model = RigidBodyTree::new(&robot.name);

        for link in &robot.links {
            model.bodies.insert(link.name.clone(), Rc::new(RefCell::new(
                RigidBody::from_link(Link::from(link.clone())))));
            model.children.insert(link.name.clone(), Vec::new());
        }

        for joint in &robot.joints {
            match model.bodies.get_mut(&joint.child.link) {
                None => {
                    error!("joint {}'s child link not found.", joint.name);
                    std::process::exit(utils::ERROR_CODE_URDF_PARSING);
                },
                Some(body) => {
                    body.borrow_mut().joint = Joint::from(joint);
                    model.joint.insert(joint.name.clone(), Rc::clone(body));
                },
            }
            model.parent.insert(joint.child.link.clone(), joint.parent.link.clone());
            match model.children.get_mut(&joint.parent.link) {
                None => {
                    error!("joint {}'s parent link not found.", joint.name);
                    std::process::exit(utils::ERROR_CODE_URDF_PARSING);
                },
                Some(children) => {
                    children.push(joint.child.link.clone());
                }
            }
        }

        for body in model.bodies.values() {
            // Specify the base body of the rigid body tree.
            // Basically, the base body is the only one that does not have
            // a parent body.
            if model.parent.get(&body.borrow().link.name).is_none() {
                // if more than one body are base candidates
                if model.base.is_some() {
                    error!("multiple base link detected.");
                    std::process::exit(utils::ERROR_CODE_URDF_PARSING);
                }
                model.base = Some(Rc::clone(body));
            }

            // Update DoF and num_fixed_body, num_non_fixed_body
            match body.borrow().joint.joint_type {
                JointType::Fixed => {
                    model.num_fixed_body += 1;
                },
                JointType::Prismatic { axis: _ } => {
                    model.num_non_fixed_body += 1;
                    model.dof += 1;
                }
                JointType::Revolute { axis: _ } => {
                    model.num_non_fixed_body += 1;
                    model.dof += 1;
                }
            }
        }

        // perform DFS to obtain the topological sort of the rigid bodies
        // and store the order in `body_id2ptr`
        match &model.base {
            None => {},
            Some(base) => {
                model.body_id2ptr.push(Rc::clone(base));
                let mut stack = Vec::new();
                stack.push(Rc::clone(base));
                while !stack.is_empty() {
                    let parent = stack.pop().unwrap();
                    if let Some(children_name_vec) = model.children.get(&parent.borrow().link.name) {
                        for child_name in children_name_vec {
                            if let Some(child_body) = model.bodies.get(child_name) {
                                stack.push(Rc::clone(child_body));
                                model.body_id2ptr.push(Rc::clone(child_body));
                            }
                        }
                    };
                }
            },
        }

        return model;
    }
}

impl fmt::Display for RigidBodyTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut table = Table::new();
        table.set_format(*format::consts::FORMAT_NO_LINESEP_WITH_TITLE);
        table.set_titles(row![
        "Idx",
        format!("Body Name ({} + {})", self.num_fixed_body, self.num_non_fixed_body),
        "Joint Name", "Joint Type", "Parent Name", "Children Name(s)"]);

        let mut index = 0;
        for body in &self.body_id2ptr {
            let body = body.borrow();
            let name = &body.link.name;
            let none = "None".to_string();
            let parent_name = self.parent.get(name).unwrap_or(&none);
            let mut children_names = "".to_string();
            match self.children.get(name) {
                None => {},
                Some(children) => {
                    for child in children {
                        children_names.push_str(&child);
                        children_names.push_str(" ");
                    }
                },
            };
            table.add_row(row![
                Cell::new(format!("{}", index).as_ref()),
                Cell::new(&body.link.name),
                Cell::new(&body.joint.name),
                Cell::new(&body.joint.joint_type.to_string()),
                Cell::new(parent_name),
                Cell::new(&children_names),
            ]);
            index += 1;
        }

        write!(f, "{}", table.to_string())
    }
}