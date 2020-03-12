use std::path::Path;
use urdf_rs::Robot;
use std::collections::HashMap;
use crate::robotics::{JointBuilder, Link, Joint};
use petgraph::Graph;
use petgraph::graph::NodeIndex;
use std::fmt;
use petgraph::dot::{Dot, Config};
use log::{info, error};
use crate::utils;

#[derive(Clone, Debug)]
pub struct RigidBodyTree {
    pub name: String,                           // name
    // contained_joints: Vec<Node<T>>,
    // movable_joints: Vec<Node<T>>,
    dof: usize,
    graph: Graph<Link, Joint>,
}

trait RigidBodyTreeModel {
    fn import();                                // import rigid body tree model from urdf
    fn load();                                  // load rigid body tree model
    fn new();                                   // create an empty rigid body tree model
    fn add_body();                              // add body to rigid body tree
    fn get_body();                              // get body handle by name
    fn add_rigid_body_subtree();                // Add subtree to rigid body tree model
}

impl RigidBodyTree {
    pub fn from_urdf_file<P>(path: P) -> Result<Self, urdf_rs::UrdfError>
    where P: AsRef<Path> {
        Ok(urdf_rs::read_file(path)?.into())
    }
}

impl From<urdf_rs::Robot> for RigidBodyTree {
    fn from(robot: Robot) -> Self {
        Self::from(&robot)
    }
}

impl<'a> From<&'a urdf_rs::Robot> for RigidBodyTree {

    fn from(robot: &urdf_rs::Robot) -> Self {
        let mut link_name2handler = HashMap::new();
        let mut model = RigidBodyTree {
            name: robot.name.clone(),
            dof: 0,
            graph: Graph::new(),
        };

        for link in &robot.links {
            let handler = model.graph.add_node(Link::from(link.clone()));
            link_name2handler.insert(link.name.clone(), handler);
        }

        for joint in &robot.joints {
            let parent = match link_name2handler.get(&joint.parent.link) {
                None => {
                    error!("joint {}'s parent link not found.", joint.name);
                    None
                },
                Some(handler) => Some(handler.clone()),
            };

            let child = match link_name2handler.get(&joint.child.link) {
                None => {
                    error!("joint {}'s child link not found.", joint.name);
                    None
                },
                Some(handler) => Some(handler.clone()),
            };

            if parent.is_none() || child.is_none() {
                std::process::exit(utils::ERROR_CODE_URDF_PARSING);
            }

            model.graph.add_edge(parent.unwrap(), child.unwrap(), Joint::from(joint));
        }

        return model;
    }
}

impl fmt::Display for RigidBodyTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", Dot::with_config(&self.graph, &[]))
    }
}