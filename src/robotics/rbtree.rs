use std::path::Path;
use urdf_rs::Robot;
use std::collections::HashMap;
use crate::robotics::*;
use std::fmt;
use log::{info, error};
use crate::utils;
use crate::math::*;
use crate::simulation::sim_model::{RenderableObject, SimScene};
use std::cell::RefCell;
use std::rc::Rc;
use prettytable::{Cell, Row, Table, format};
use kiss3d::scene::SceneNode;
use failure::_core::cmp::max;
use std::cmp::min;


#[derive(Clone)]
pub struct RigidBodyTree {
    name: String,                                       // name
    dof: usize,                                         // degree of freedom
    num_fixed_body: usize,                              // number of bodies with fixed joint
    num_non_fixed_body: usize,                          // number of bodies with non fixed joint
    base: Option<Rc<RefCell<RigidBody>>>,               // root of the tree
    bodies: HashMap<String, Rc<RefCell<RigidBody>>>,    // rigid bodies
    parent: HashMap<String, String>,                    // name of the parent rigid body
    children: HashMap<String, Vec<String>>,             // name of the child rigid bodies
    joint: HashMap<String, Rc<RefCell<RigidBody>>>,     // mapping from joint name to rigid body
    body_id2ptr: Vec<Rc<RefCell<RigidBody>>>,           // mapping from id to rigid body
    scene_id2ptr: Vec<Vec<SceneNode>>,                  // mapping from id to rigid body scene node
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
            scene_id2ptr: Vec::new(),
        }
    }

    /// Get body from RigidBodyTree
    pub fn get_body(&self, name: &str) -> RigidBody {
        if let Some(body) = self.bodies.get(name) {
            let body = body.borrow();
            return body.clone();
        } else {
            error!("body '{}' not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }

    /// Get joint from RigidBodyTree by name
    pub fn get_joint(&self, name: &str) -> Option<Joint> {
        if let Some(body) = self.joint.get(name) {
            let body = body.borrow();
            return body.joint.clone();
        } else {
            error!("joint '{}' not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }

    /// Get joint ptr from RigidBodyTree by name
    pub fn get_body_ptr(&self, name: &str) -> Rc<RefCell<RigidBody>> {
        if let Some(body) = self.bodies.get(name) {
            return Rc::clone(body);
        } else {
            error!("body '{}' not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }

    /// Add a subtree to RigidBodyTree
    fn add_rigid_body_subtree(&mut self) {
        unimplemented!()
    }

    /// Number of bodies
    pub fn num_body(&self) -> usize {
        self.bodies.len()
    }

    /// Number of bodies with fixed joint
    pub fn num_fixed_body(&self) -> usize {
        self.num_fixed_body
    }

    /// Number of bodies with non-fixed joint (e.g. revolute, prismatic)
    pub fn num_non_fixed_body(&self) -> usize {
        self.num_non_fixed_body
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
        let nb = self.num_body();
        let mut Ic = vec![Matrix6f::zeros(); nb];  // composite-rigid-body inertia
        let mut xforms = vec![Matrix6f::zeros(); nb];  // spatial transform from parent of body i to body i
        let nv = self.num_dof();
        let mut H = MatrixDDf::zeros(nv, nv);

        // preparation
        let mut k = 0;
        for i in 0..nb {
            let body = self.body_id2ptr[i].borrow();
            Ic[i] = body.link.inertial.spatial_inertia.clone();
            let pnum = body.qpos_dof();
            let qi = body.get_qpos_from_vec(qpos, k);
            k += pnum;
            let tform = body.tform_body2parent(qi);
            let tform_inv = tform_inv(tform);
            xforms[i] = tform_to_spatial_xform(tform_inv);
        }

        // main loop
        for i in (0..nb).rev() {
            let body = self.body_id2ptr[i].borrow();
            if let Some(joint) = &body.joint {
                let pid = body.parent_index.unwrap();
                Ic[pid] = Ic[pid] + xforms[i].transpose() * Ic[i] * xforms[i];
            }

            if let Some(joint) = &body.joint {
                let a = body.qvel_dof_map();
                if a.1 > a.0 {
                    let si = &joint.screw_axis;
                    let fi = Ic[i] * si;
                    H.slice_mut((a.0, a.0), (a.1 - a.0, a.1 - a.0))
                        .copy_from(&(si.transpose() * fi));

                    let mut fi = xforms[i].transpose() * fi;
                    if let Some(j_tmp) = body.parent_index {
                        let mut j = j_tmp;
                        let mut body_j = self.body_id2ptr[j].borrow();
                        while let Some(joint_j) = &body_j.joint {
                            let sj = &joint_j.screw_axis;
                            let b = body_j.qvel_dof_map();
                            if b.0 < b.1 {
                                let mass_ji = sj.transpose() * fi;
                                H.slice_mut((b.0, a.0), (b.1 - b.0, a.1 - a.0))
                                    .copy_from(&mass_ji);
                                H.slice_mut((a.0, b.0), (a.1 - a.0, b.1 - b.0))
                                    .copy_from(&mass_ji.transpose());
                            }
                            fi = xforms[j].transpose() * fi;

                            j = body_j.parent_index.unwrap();
                            body_j = self.body_id2ptr[j].borrow();
                        }
                    }
                }
            }
        }

        return H;
    }

    /// Get home configuration
    pub fn home_configuration(&self) -> VectorDf {
        let qpos = VectorDf::zeros(self.dof);
        return qpos;
    }

    /// Get body name by it ID in rigid body tree.
    pub fn body_name(&self, id: usize) -> String {
        self.body_id2ptr[id].borrow().link.name.to_string()
    }

    /// Set position and orientation of bodies registered in the scene.
    pub fn render(&mut self, qpos: &VectorDf) {
        let tforms = self.forward_kinematics(qpos);
        for (i, scene_nodes) in self.scene_id2ptr.iter_mut().enumerate() {
            let name = self.body_id2ptr[i].borrow().link.name.to_string();
            for node in scene_nodes.iter_mut() {
                let tvec = Translation3f32::new(
                    tforms[i][(0, 3)] as f32, tforms[i][(1, 3)] as f32, tforms[i][(2, 3)] as f32);
                let rotm = Rotation3f32::from_matrix_unchecked(Matrix3f32::new(
                    tforms[i][(0, 0)] as f32, tforms[i][(0, 1)] as f32, tforms[i][(0, 2)] as f32,
                    tforms[i][(1, 0)] as f32, tforms[i][(1, 1)] as f32, tforms[i][(1, 2)] as f32,
                    tforms[i][(2, 0)] as f32, tforms[i][(2, 1)] as f32, tforms[i][(2, 2)] as f32,
                ));
                let quat = UnitQuat4f32::from_rotation_matrix(&rotm);
                let isometry = Isometry3f32::from_parts(tvec, quat);
                node.set_local_transformation(isometry);
            }
        }
    }

    /// Return body index in the rigid body tree given body name.
    pub fn body_index_from_name(&self, name: &str) -> usize {
        if let Some(body) = self.bodies.get(name) {
            body.borrow().index
        } else {
            error!("body name {} not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }

    /// Return the index of parent body
    pub fn parent_index(&self, id: usize) -> Option<usize> {
        if id >= self.body_id2ptr.len() {
            error!("body id {} not found.", id);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }

        let parent = self.body_id2ptr[id].borrow().parent_index;
        return parent;
    }

    /// Compute all transformation matrix from body frame `{B}` to world
    /// frame `{W}`.
    pub fn forward_kinematics(&self, qpos: &VectorDf) -> Vec<Matrix4f> {
        let n = self.num_body();
        let mut tforms = vec![Matrix4f::identity(); n];
        let mut k = 0;

        for i in 0..n {
            let body = self.body_id2ptr[i].borrow();
            let pnum = body.qpos_dof();
            let qi = body.get_qpos_from_vec(qpos, k);
            tforms[i] = body.tform_body2parent(qi);

            k = k + pnum;
            if let Some(parent_idx) = body.parent_index {
                tforms[i] = tforms[parent_idx] * tforms[i];
            }
        }

        return tforms;
    }

    /// Compute the shortest kinematic path from body `from` to body
    /// `to` in the rigid body tree.
    pub fn kinematics_tree_path(&self, from: &str, to: &str) -> Vec<usize> {
        let id_from = self.body_index_from_name(from);
        let id_to = self.body_index_from_name(to);

        let mut path = Vec::new();
        let id_larger = max(id_to, id_from);
        let id_smaller = min(id_to, id_from);
        let mut curr = id_larger;
        path.push(curr);
        while curr > id_smaller {
            if let Some(parent) = self.parent_index(curr) {
                curr = parent;
                path.push(curr);
            } else {
                break;
            }
        }

        if curr != id_smaller {
            return Vec::new();
        }

        if id_to > id_from {
            path.reverse();
        }
        return path;
    }

    /// Get the transform T that converts points originally expressed
    /// in `{from}` frame to `{to}` frame
    pub fn get_transform(&self, qpos: &VectorDf, from: &str, to: &str) -> Matrix4f {
        let tforms = self.forward_kinematics(qpos);

        let id_from = self.body_index_from_name(from);
        let tform_from2world = tforms[id_from];
        let id_to = self.body_index_from_name(to);
        let tform_to2world = tforms[id_to];
        let tform_world2to = tform_inv(tform_to2world);
        return tform_world2to * tform_from2world;
    }

    /// Get the transform T that converts points originally expressed
    /// in `{from}` frame to world frame `{W}`
    pub fn get_transform_to_world(&self, qpos: &VectorDf, from: &str) -> Matrix4f {
        let tforms  = self.forward_kinematics(qpos);
        let from_id = self.body_index_from_name(from);
        return tforms[from_id];
    }


    /// Get all body names
    pub fn get_body_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        for body in self.bodies.values() {
            names.push(body.borrow().link.name.clone());
        }
        return names;
    }

    /// Get base name
    pub fn get_base_name(&self) -> Option<String> {
        if let Some(base) = &self.base {
            Some(base.borrow().link.name.clone())
        } else {
            None
        }
    }

    pub fn register_scene(&mut self, scene: &mut SimScene) {
        self.scene_id2ptr = vec![Vec::new(); self.bodies.len()];

        for (i, body) in self.body_id2ptr.iter().enumerate() {
            for visual in &body.borrow().link.visuals {
                match &visual.geometry {
                    Geometry::Mesh { filename, scale, mesh } => {
                        let mut h = scene.window.add_mesh(Rc::clone(mesh), scale.clone_owned());
                        h.set_color(visual.material.color.r,
                                    visual.material.color.g,
                                    visual.material.color.b);
                        h.enable_backface_culling(false);
                        self.scene_id2ptr[i].push(h.clone());
                    },
                    _ => {},
                }
            }
        }
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

impl fmt::Display for RigidBodyTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut table = Table::new();
        table.set_format(*format::consts::FORMAT_NO_LINESEP_WITH_TITLE);
        table.set_titles(row![
        "Idx",
        format!("Body Name ({} + {})", self.num_fixed_body, self.num_non_fixed_body),
        "Joint Name", "Joint Type", "qpos Map", "qvel Map", "Parent Name", "Children Name(s)"]);

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
                Cell::new(&body.name()),
                Cell::new(&body.joint_name()),
                Cell::new(&body.joint_type_name()),
                Cell::new(format!("({}, {})",
                              body.qpos_dof_map().0,
                              body.qpos_dof_map().1).as_ref()),
                Cell::new(format!("({}, {})",
                              body.qvel_dof_map().0,
                              body.qvel_dof_map().1).as_ref()),
                Cell::new(parent_name),
                Cell::new(&children_names),
            ]);
            index += 1;
        }

        write!(f, "{}", table.to_string())
    }
}