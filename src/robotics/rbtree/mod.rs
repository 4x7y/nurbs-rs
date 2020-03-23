use std::cell::RefCell;
use std::rc::Rc;
use std::collections::HashMap;
use crate::math::Vector3f;
use kiss3d::scene::SceneNode;
use crate::robotics::RigidBody;

mod import;
mod display;
mod property;
mod kinematics;
mod dynamics;

type RigidBodyPtr = Rc<RefCell<RigidBody>>;

#[derive(Clone)]
pub struct RigidBodyTree {
    name: String,                                  // name
    dof: usize,                                    // degree of freedom
    num_fixed_body: usize,                         // number of bodies with fixed joint
    num_non_fixed_body: usize,                     // number of bodies with non fixed joint
    base: Option<RigidBodyPtr>,                    // root of the tree
    body_name2ptr: HashMap<String, RigidBodyPtr>,  // rigid bodies
    parent: HashMap<String, String>,               // name of the parent rigid body
    children: HashMap<String, Vec<String>>,        // name of the child rigid bodies
    joint: HashMap<String, RigidBodyPtr>,          // mapping from joint name to rigid body
    bodies: Vec<RigidBodyPtr>,                     // mapping from id to rigid body
    scene_id2ptr: Vec<Vec<SceneNode>>,             // mapping from id to rigid body scene node
    gravity: Vector3f,                             // gravitational acceleration
}