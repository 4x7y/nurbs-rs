extern crate rand;

use crate::math::*;
use crate::robotics::*;
use std::collections::HashMap;
use rand::Rng;

pub struct RobotModel {
    pub nq: usize,                    // dimension of generalized coordinates
    pub nv: usize,                    // dimension of generalized velocities
    pub nbody: usize,                 // number of bodies
    pub gravity: Vector3f,            // gravitational acceleration experienced by robot

    // rigid body tree
    pub bodies: Vec<RigidBody>,       // list of rigid bodies
    pub parent: Vec<Option<usize>>,   // parent link index
    pub child: Vec<Vec<usize>>,       // children link index

    // inertia
    pub bimm_vecs: Vec<Vector6f>,     // list of body inertia vectors  (body frame)
    pub bimm_mats: Vec<Matrix3f>,     // list of body inertia matrices (body frame)
    pub pimm_vecs: Vec<Vector3f>,     // list of body inertia matrices (principal frame)
    pub brot_prcp: Vec<Matrix3f>,     // rotation matrices from principal frame to body

    // limits on the qpos, qvel, qacc
    pub qpos_ulmt: Vec<Scalar>,          // upper limits of qpos
    pub qpos_llmt: Vec<Scalar>,          // lower limits of qpos
    pub qvel_ulmt: Vec<Scalar>,          // upper limits of qvel
    pub qvel_llmt: Vec<Scalar>,          // lower limits of qvel
    pub qacc_ulmt: Vec<Scalar>,          // upper limits of qacc
    pub qacc_llmt: Vec<Scalar>,          // lower limits of qacc

    // home configuration
    pub qpos_home: Vec<Scalar>,          // home configuration

    // names
    pub body_name2id: HashMap<String, usize>,
    pub body_id2name: Vec<String>,
    pub jnts_name2id: HashMap<String, usize>,
    pub jnts_id2name: Vec<String>,
}

pub struct RobotState {
    // state
    pub qpos: VectorDf,               // generalized coordinates position                   (nq x 1)
    pub qvel: VectorDf,               // generalized coordinates velocity                   (nv x 1)

    // dynamics
    pub qacc: VectorDf,               // acceleration                                       (nv x 1)
    pub act_dot: VectorDf,            // time-derivative of actuator activation             (na x 1)
    pub mm: MatrixDDf,                // total inertia                                      (nv x nv)
    pub crb: VectorDf,                // com-based composite inertia and mass               (nbody x 10）

    // control
    pub ctrl: VectorDf,               // control input to the actuators                     (nu x 1)
    pub qfrc_applied: VectorDf,       // applied generalized force                          (nv x 1)
    pub xfrc_applied: VectorDf,       // applied Cartesian force/torque                     (nbody x 6)
}

impl RobotModel {

    /// Create an empty robot model
    pub fn new() -> Self {
        RobotModel {
            nq: 0,
            nv: 0,
            nbody: 0,
            gravity: Vector3f::new(0., 0., -9.81),
            bodies: vec![],
            parent: vec![],
            child: vec![],
            bimm_vecs: vec![],
            bimm_mats: vec![],
            pimm_vecs: vec![],
            brot_prcp: vec![],
            qpos_ulmt: vec![],
            qpos_llmt: vec![],
            qvel_ulmt: vec![],
            qvel_llmt: vec![],
            qacc_ulmt: vec![],
            qacc_llmt: vec![],
            qpos_home: vec![],
            body_name2id: HashMap::new(),
            body_id2name: vec![],
            jnts_name2id: HashMap::new(),
            jnts_id2name: vec![],
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

    fn random_configuration(&self) -> Vec<Scalar> {
        let mut rng = rand::thread_rng();
        let mut qpos = self.home_configuration();
        for i in 0..self.nv {
            qpos[i] = rng.gen_range(self.qpos_llmt[i], self.qpos_ulmt[i]);
        }
        return qpos;
    }

    fn home_configuration(&self) -> Vec<Scalar> {
        return self.qpos_home.clone();
    }

    fn geometric_jacobian(&self) -> MatrixDDf {
        unimplemented!()
    }
}


