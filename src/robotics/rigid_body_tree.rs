use std::path::Path;
use urdf_rs::Robot;
use std::collections::HashMap;
use crate::robotics::{JointBuilder, Link, Joint};
use petgraph::Graph;
use petgraph::graph::NodeIndex;
use std::fmt;
use petgraph::dot::{Dot, Config};

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

    // #[allow(clippy::needless_pass_by_value)]
    // pub fn from_root(root_joint: Node<T>) -> Self {
    //     let contained_joints = root_joint.iter_descendants().collect::<Vec<_>>();
    //     let movable_joints = contained_joints
    //         .iter()
    //         .filter(|joint| joint.joint().is_movable())
    //         .cloned()
    //         .collect::<Vec<_>>();
    //
    //     Self {
    //         name: "".to_string(),
    //         dof: movable_joints.len(),
    //         contained_joints: contained_joints,
    //         movable_joints: movable_joints,
    //     }
    // }
}

impl From<urdf_rs::Robot> for RigidBodyTree {
    fn from(robot: Robot) -> Self {
        Self::from(&robot)
    }
}

pub const ROOT_JOINT_NAME: &str = "root";

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
                None => panic!("joint {}'s parent link not found.", joint.name),
                Some(handler) => handler.clone(),
            };

            let child = match link_name2handler.get(&joint.child.link) {
                None => panic!("joint {}'s child link not found.", joint.name),
                Some(handler) => handler.clone(),
            };

            model.graph.add_edge(parent, child, Joint::from(joint));
        }

        return model;
    }
}

impl fmt::Display for RigidBodyTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", Dot::with_config(&self.graph, &[]))
    }
}