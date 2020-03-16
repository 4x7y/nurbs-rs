use std::path::Path;
use urdf_rs::Robot;
use std::collections::HashMap;
use crate::robotics::{JointBuilder, Link, Joint};
use petgraph::Graph;
use petgraph::graph::{NodeIndex, EdgeIndex};
use std::fmt;
use petgraph::dot::{Dot, Config};
use log::{info, error};
use crate::utils;
use crate::math::*;

#[derive(Clone, Debug)]
pub struct RigidBodyTree {
    pub name: String,                           // name
    dof: usize,                                 // degree of freedom
    graph: Graph<Link, Joint>,                  // graph based representation
    jnts_name2hdl: HashMap<String, EdgeIndex>,  // mapping from joint name to its graph handler
    link_name2hdl: HashMap<String, NodeIndex>,  // mapping from link name to its graph handler
    num_fixed_body: usize,
    num_non_fixed_body: usize,
    qpos_dof_map: HashMap<String, usize>,
    qvel_dof_map: HashMap<String, usize>,
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
            graph: Graph::new(),
            jnts_name2hdl: HashMap::new(),
            link_name2hdl: HashMap::new(),
            num_fixed_body: 0,
            num_non_fixed_body: 0,
            qpos_dof_map: HashMap::new(),
            qvel_dof_map: HashMap::new(),
        }
    }

    /// Add link to RigidBodyTree
    pub fn add_link(&mut self, link: &Link, joint: &Joint, parent: &String) {
        let parent = match self.link_name2hdl.get(parent) {
            None => {
                error!("parent link '{}' not found.", parent);
                std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
            },
            Some(handler) => handler.clone(),
        };
        let child = self.graph.add_node(link.clone());
        let edge  = self.graph.add_edge(parent, child, joint.clone());

        {
            self.link_name2hdl.insert(link.name.clone(), child);
            self.jnts_name2hdl.insert(joint.name.clone(), edge);
        }
    }

    /// Get link from RigidBodyTree
    pub fn get_link(&self, name: &String) -> Link {
        let handler = match self.link_name2hdl.get(name) {
            None => {
                error!("link '{}' not found.", name);
                std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
            },
            Some(handler) => handler.clone(),
        };

        return self.graph.node_weight(handler).unwrap().clone();
    }

    /// Get joint from RigidBodyTree by name
    pub fn get_joint(&self, name: &String) -> Joint {
        let handler = match self.jnts_name2hdl.get(name) {
            None => {
                error!("joint '{}' not found.", name);
                std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
            },
            Some(handler) => handler.clone(),
        };

        return self.graph.edge_weight(handler).unwrap().clone();
    }

    /// Add a subtree to RigidBodyTree
    fn add_rigid_body_subtree(&mut self) {
        unimplemented!()
    }

    /// Number of links
    pub fn num_link(&self) -> usize {
        return self.graph.node_count();
    }

    /// Number of joints
    pub fn num_joint(&self) -> usize {
        return self.graph.edge_count();
    }

    /// Compute the mass matrix, `M`, of the robot in the configuration `q`
    pub fn mass_matrix(&self, qpos: &VectorDf) -> MatrixDDf {
        let nb = self.num_link();
        let mut crb_inertia = vec![Matrix6f::zeros(); nb]; // composite-rigid-body inertia
        let mut xforms      = vec![Matrix4f::zeros(); nb]; // spatial transform from parent of body i to body i
        let nv = self.num_joint();
        let mass_matrix = MatrixDDf::zeros(nv, nv);
        let lambda_ = vec![0.; nb];
        let lambda  = vec![0.; nv];

        // preparation
        unimplemented!();

        return mass_matrix;
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
            let handler = model.graph.add_node(Link::from(link.clone()));
            model.link_name2hdl.insert(link.name.clone(), handler);
        }

        for joint in &robot.joints {
            let parent = match model.link_name2hdl.get(&joint.parent.link) {
                None => {
                    error!("joint {}'s parent link not found.", joint.name);
                    None
                },
                Some(handler) => Some(handler.clone()),
            };

            let child = match model.link_name2hdl.get(&joint.child.link) {
                None => {
                    error!("joint {}'s child link not found.", joint.name);
                    None
                },
                Some(handler) => Some(handler.clone()),
            };

            if parent.is_none() || child.is_none() {
                std::process::exit(utils::ERROR_CODE_URDF_PARSING);
            }

            let handle = model.graph.add_edge(
                parent.unwrap(), child.unwrap(), Joint::from(joint));
            model.jnts_name2hdl.insert(joint.name.clone(), handle);
        }

        return model;
    }
}

impl fmt::Display for RigidBodyTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", Dot::with_config(&self.graph, &[]))
    }
}