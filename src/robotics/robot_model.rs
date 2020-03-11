extern crate rand;

use crate::math::*;
use crate::robotics::*;
use std::collections::HashMap;
use rand::Rng;

pub struct RobotModel {
    pub nq: usize,                        // dimension of generalized coordinates = dim(qpos)
    pub nv: usize,                        // dimension of generalized velocities = dim(qvel)
    pub nu: usize,                        // number of actuators/controls = dim(ctrl)
    pub na: usize,                        // number of activation states = dim(act)
    pub nbody: usize,                     // number of bodies
    pub gravity: Vector3f,                // gravitational acceleration experienced by robot

    // rigid body tree
    pub bodies: Vec<RigidBody>,           // list of rigid bodies
    pub parent: Vec<Option<usize>>,       // parent link index
    pub child: Vec<Vec<usize>>,           // children link index

    // inertia
    pub bimm_vecs: Vec<Vector6f>,         // list of body inertia vectors  (body frame)
    pub bimm_mats: Vec<Matrix3f>,         // list of body inertia matrices (body frame)
    pub pimm_vecs: Vec<Vector3f>,         // list of body inertia matrices (principal frame)
    pub brot_prcp: Vec<Matrix3f>,         // rotation matrices from principal frame to body

    // limits on the qpos, qvel, qacc
    pub qpos_ulmt: VectorDf,              // upper limits of qpos
    pub qpos_llmt: VectorDf,              // lower limits of qpos
    pub qvel_ulmt: VectorDf,              // upper limits of qvel
    pub qvel_llmt: VectorDf,              // lower limits of qvel
    pub qacc_ulmt: VectorDf,              // upper limits of qacc
    pub qacc_llmt: VectorDf,              // lower limits of qacc

    // home configuration
    pub qpos_home: VectorDf,              // home configuration

    // names
    pub body_name2id: HashMap<String, usize>,
    pub body_id2name: Vec<String>,
    pub jnts_name2id: HashMap<String, usize>,
    pub jnts_id2name: Vec<String>,

    // screw
    pub tform_to_prev: Vec<Matrix4f>,     // list of link frames {i} relative to {i-1} at the home
                                          // position
    pub spatial_inertia: Vec<Matrix6f>,   // spatial inertia matrices Gi of the links
    pub screw: Vec<Vector6f>,             // screw axes Si of the joints in a space frame,
                                          // in the format of a matrix with axes as the columns
}

impl RobotModel {

    /// Create an empty robot model
    pub fn new(nq: usize, nv: usize, nu: usize, na: usize, nbody: usize) -> Self {
        RobotModel {
            nq: nq,
            nv: nv,
            nu: nu,
            na: na,
            nbody: nbody,
            gravity: Vector3f::new(0., 0., -9.81),
            bodies: vec![],
            parent: vec![],
            child: vec![],
            bimm_vecs: vec![],
            bimm_mats: vec![],
            pimm_vecs: vec![],
            brot_prcp: vec![],
            qpos_ulmt: VectorDf::repeat(nq, INFINITY),
            qpos_llmt: VectorDf::repeat(nq, NEG_INFINITY),
            qvel_ulmt: VectorDf::repeat(nv, INFINITY),
            qvel_llmt: VectorDf::repeat(nv, NEG_INFINITY),
            qacc_ulmt: VectorDf::repeat(nv, INFINITY),
            qacc_llmt: VectorDf::repeat(nv, NEG_INFINITY),
            qpos_home: VectorDf::zeros(nq),
            body_name2id: HashMap::new(),
            body_id2name: vec![],
            jnts_name2id: HashMap::new(),
            jnts_id2name: vec![],
            tform_to_prev: vec![],
            spatial_inertia: vec![],
            screw: vec![]
        }
    }

    pub fn number_of_joint(&self) -> usize {
        return self.nv;
    }

    pub fn number_of_body(&self) -> usize {
        return self.nbody;
    }

    pub fn ximm(&self, q: &VectorDf) -> MatrixDDf {
        return crba(self, &q);
    }

    pub fn ximm_all(&self, q: &VectorDf) -> Vec<Matrix3f> {
        unimplemented!()
    }

    pub fn add_body(&self, body: RigidBody) {
        unimplemented!();
    }

    pub fn add_subtree(&self, subtree: RigidBodyTree) {
        unimplemented!();
    }
}

impl Dynamics for RobotModel {
    fn center_of_mass() {
        unimplemented!()
    }

    fn mass_matrix() {
        unimplemented!()
    }

    fn forward_dynamics() {
        unimplemented!()
    }

    fn inverse_dynamics() {
        unimplemented!()
    }

    fn velocity_product() {
        unimplemented!()
    }

    fn gravity_torque() {
        unimplemented!()
    }
}

impl Kinematics for RobotModel {
    /// Get transform matrix between body frames
    fn get_transform(&self, qpos: VectorDf, body_from: String, body_to: String) -> Matrix4f {
        unimplemented!()
    }

    fn random_configuration(&self) -> VectorDf {
        let mut rng = rand::thread_rng();
        let mut qpos = self.home_configuration();
        for i in 0..self.nv {
            qpos[i] = rng.gen_range(self.qpos_llmt[i], self.qpos_ulmt[i]);
        }
        return qpos;
    }

    fn home_configuration(&self) -> VectorDf {
        return self.qpos_home.clone();
    }

    fn geometric_jacobian(&self) -> MatrixDDf {
        unimplemented!()
    }
}


