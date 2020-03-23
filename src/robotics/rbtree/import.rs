use crate::robotics::*;
use urdf_rs::Robot;
use std::cell::RefCell;
use std::rc::Rc;
use log::error;
use crate::utils;

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
                RigidBody::from_link(Link::from(link.clone()), true))));
            model.children.insert(link.name.clone(), Vec::new());
        }

        for joint in &robot.joints {
            match model.bodies.get_mut(&joint.child.link) {
                None => {
                    error!("joint {}'s child link not found.", joint.name);
                    std::process::exit(utils::ERROR_CODE_URDF_PARSING);
                },
                Some(body) => {
                    body.borrow_mut().joint = Some(Joint::from(joint));
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

        let mut qvel_dof = 0;
        let mut qpos_dof = 0;
        for body_ptr in model.bodies.values() {
            let body = body_ptr.borrow_mut();

            // Specify the base body of the rigid body tree.
            // Basically, the base body is the only one that does not have
            // a parent body.
            if model.parent.get(&body.link.name).is_none() {
                // if more than one body are base candidates
                if model.base.is_some() {
                    error!("multiple base link detected.");
                    std::process::exit(utils::ERROR_CODE_URDF_PARSING);
                }
                model.base = Some(Rc::clone(body_ptr));
            }
        }

        // perform DFS to obtain the topological sort of the rigid bodies
        // and store the order in `body_id2ptr`
        if let Some(base) = &model.base {
            model.body_id2ptr.push(Rc::clone(base));
            base.borrow_mut().index = 0;
            base.borrow_mut().parent_index = None;

            let mut stack = Vec::new();
            stack.push(Rc::clone(base));
            while !stack.is_empty() {
                let parent = stack.pop().unwrap();
                if let Some(children_name_vec) = model.children.get(&parent.borrow().link.name) {
                    for child_name in children_name_vec {
                        if let Some(child_body) = model.bodies.get(child_name) {
                            stack.push(Rc::clone(child_body));
                            let index = model.body_id2ptr.len();
                            model.body_id2ptr.push(Rc::clone(child_body));
                            let mut child_body = child_body.borrow_mut();
                            child_body.parent_index = Some(parent.borrow().index);
                            child_body.index = index;
                        }
                    }
                };
            }
        }

        // set qpos_dof_map and qvel_dof_map
        for body_ptr in model.body_id2ptr.iter() {
            let mut body = body_ptr.borrow_mut();
            // Update DoF and num_fixed_body, num_non_fixed_body
            match body.joint_type() {
                None => {
                    body.qvel_dof_map = (qvel_dof, qvel_dof);
                    body.qpos_dof_map = (qpos_dof, qpos_dof);
                },
                Some(JointType::Fixed) => {
                    model.num_fixed_body += 1;
                    body.qvel_dof_map = (qvel_dof, qvel_dof);
                    body.qpos_dof_map = (qpos_dof, qpos_dof);
                },
                Some(JointType::Prismatic { axis: _ }) => {
                    model.num_non_fixed_body += 1;
                    model.dof += 1;
                    body.qvel_dof_map = (qvel_dof, qvel_dof + 1);
                    body.qpos_dof_map = (qpos_dof, qpos_dof + 1);
                    qpos_dof += 1;
                    qvel_dof += 1;
                }
                Some(JointType::Revolute { axis: _ }) => {
                    model.num_non_fixed_body += 1;
                    model.dof += 1;
                    body.qvel_dof_map = (qvel_dof, qvel_dof + 1);
                    body.qpos_dof_map = (qpos_dof, qpos_dof + 1);
                    qpos_dof += 1;
                    qvel_dof += 1;
                }
            }
        }

        return model;
    }
}
