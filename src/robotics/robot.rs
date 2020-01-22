use crate::math::vector::*;

pub struct Robot {
    // state
    pub qpos: VectorNf,               // generalized coordinates position         (nq x 1)
    pub qvel: VectorNf,               // generalized coordinates velocity         (nv x 1)

    // control
    pub ctrl: VectorNf,
    pub qfrc_applied: VectorNf,
    pub xfrc_applied: VectorNf,

    // dynamics
    pub qacc: VectorNf,               // acceleration                             (nv x 1)
    pub act_dot: VectorNf,            // time-derivative of actuator activation   (na x 1)
}