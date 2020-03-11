use crate::math::*;
use crate::robotics::*;

pub struct RobotState {
    // state
    pub qpos: VectorDf,               // generalized coordinates position              (nq x 1)
    pub qvel: VectorDf,               // generalized coordinates velocity              (nv x 1)

    // dynamics
    pub qacc: VectorDf,               // acceleration                                  (nv x 1)
    pub act_dot: VectorDf,            // time-derivative of actuator activation        (na x 1)
    pub mm: MatrixDDf,                // total inertia                                 (nv x nv)
    pub crb: Vec<VectorDf>,           // com-based composite inertia and mass          (nbody x 10）

    // control
    pub ctrl: VectorDf,               // control input to the actuators                (nu x 1)
    pub qfrc_applied: VectorDf,       // applied generalized force                     (nv x 1)
    pub xfrc_applied: Vec<VectorDf>,  // applied Cartesian force/torque                (nbody x 6)
}

impl RobotState {
    /// Create an empty robot state
    pub fn new(nq: usize, nv: usize, na: usize, nu: usize, nbody: usize) -> Self {
        RobotState {
            qpos: VectorDf::zeros(nq),
            qvel: VectorDf::zeros(nv),
            qacc: VectorDf::zeros(nv),
            act_dot: VectorDf::zeros(na),
            mm: MatrixDDf::zeros(nv, nv),
            crb: vec![VectorDf::zeros(10); nbody],
            ctrl: VectorDf::zeros(nu),
            qfrc_applied: VectorDf::zeros(nv),
            xfrc_applied: vec![VectorDf::zeros(6); nbody],
        }
    }

    /// Create an empty robot state from a RobotModel instance
    pub fn from_robot_model(model: &RobotModel) -> Self {
        RobotState::new(model.nq, model.nv, model.na, model.nu, model.nbody)
    }
}