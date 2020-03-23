use crate::math::*;
use crate::utils;
use crate::robotics::*;
use std::path::Path;
use std::collections::HashMap;
use std::cell::RefCell;
use std::rc::Rc;
use log::error;
use crate::simulation::sim_model::SimScene;

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
            body_name2ptr: HashMap::new(),
            parent: HashMap::new(),
            children: HashMap::new(),
            joint: HashMap::new(),
            bodies: Vec::new(),
            scene_id2ptr: Vec::new(),
            gravity: Vector3f::new(0., 0., -9.78),
        }
    }

    /// Set gravitational acceleration for the rigid body tree.
    pub fn set_gravity(&mut self, gravity: Vector3f) {
        self.gravity = gravity;
    }

    /// Get gravitational acceleration.
    pub fn get_gravity(&self) -> Vector3f {
        self.gravity
    }

    /// Add a body to the robot
    pub fn add_body(&mut self, body: RigidBody) {
        unimplemented!()
    }

    /// Get body from RigidBodyTree
    pub fn get_body(&self, name: &str) -> RigidBody {
        if let Some(body) = self.body_name2ptr.get(name) {
            let body = body.borrow();
            return body.clone();
        } else {
            error!("body '{}' not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }

    /// Replace a body in the robot
    pub fn replace_body(&self, name: &str, body: RigidBody) {
        unimplemented!()
    }


    /// Get joint from RigidBodyTree by name
    pub fn get_joint(&self, name: &str) -> Joint {
        if let Some(body) = self.joint.get(name) {
            let body = body.borrow();
            return body.joint.clone();
        } else {
            error!("joint '{}' not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }


    /// Replace the joint of one of robot's body
    pub fn replace_joint(&self, name: &str, joint: Joint) {
        unimplemented!()
    }

    /// Get joint ptr from RigidBodyTree by name
    pub fn get_body_ptr(&self, name: &str) -> Rc<RefCell<RigidBody>> {
        if let Some(body) = self.body_name2ptr.get(name) {
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

    /// Get home configuration
    pub fn home_configuration(&self) -> VectorDf {
        let qpos = VectorDf::zeros(self.dof);
        return qpos;
    }

    /// Get random configuration
    pub fn random_configuration(&self) -> VectorDf {
        unimplemented!()
    }

    /// Compute the center of mass position and Jacobian.
    ///
    /// Computes the center of mass position of ROBOT at the specified joint
    /// configuration `qpos` relative to the base frame.
    pub fn center_of_mass(&self, qpos: &VectorDf) -> Vector3f {
        unimplemented!()
    }

    /// Get body name by it ID in rigid body tree.
    pub fn body_name(&self, id: usize) -> String {
        self.bodies[id].borrow().link.name.to_string()
    }

    /// Set position and orientation of bodies registered in the scene.
    pub fn render(&mut self, qpos: &VectorDf) {
        let tforms = self.forward_kinematics(qpos);
        for (i, scene_nodes) in self.scene_id2ptr.iter_mut().enumerate() {
            let name = self.bodies[i].borrow().link.name.to_string();
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
        if let Some(body) = self.body_name2ptr.get(name) {
            body.borrow().index
        } else {
            error!("body name {} not found.", name);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }
    }

    /// Return the index of parent body
    pub fn parent_index(&self, id: usize) -> Option<usize> {
        if id >= self.bodies.len() {
            error!("body id {} not found.", id);
            std::process::exit(utils::ERROR_CODE_RIGID_BODY_TREE);
        }

        let parent = self.bodies[id].borrow().parent_index;
        return parent;
    }



    /// Get all body names
    pub fn get_body_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        for body in self.body_name2ptr.values() {
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
        self.scene_id2ptr = vec![Vec::new(); self.num_body()];

        for (i, body) in self.bodies.iter().enumerate() {
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
