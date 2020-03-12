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

#[derive(Clone, Debug)]
pub struct RigidBodyTree {
    pub name: String,                           // name
    dof: usize,                                 // degree of freedom
    graph: Graph<Link, Joint>,                  // graph based representation
    jnts_name2hdl: HashMap<String, EdgeIndex>,  // mapping from joint name to its graph handler
    link_name2hdl: HashMap<String, NodeIndex>,  // mapping from link name to its graph handler
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

    /// Add a subtree to RigidBodyTree
    fn add_rigid_body_subtree(&mut self) {
        unimplemented!()
    }
}

impl From<urdf_rs::Robot> for RigidBodyTree {
    fn from(robot: Robot) -> Self {
        Self::from(&robot)
    }
}

impl<'a> From<&'a urdf_rs::Robot> for RigidBodyTree {

    fn from(robot: &urdf_rs::Robot) -> Self {
        let mut model = RigidBodyTree {
            name: robot.name.clone(),
            dof: 0,
            graph: Graph::new(),
            jnts_name2hdl: HashMap::new(),
            link_name2hdl: HashMap::new(),
        };

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