
use std::f32;
use crate::utils::*;

#[derive(Debug, Clone)]
pub struct PIDController {
    pub kp: f32,            // proportional gain
    pub ki: f32,            // integral gain
    pub kd: f32,            // differential gain

    pub qpos_dsr: f32,      // desired position
    pub qvel_dsr: f32,      // desired velocity
    pub qacc_dsr: f32,      // desired acceleration

    pub output: f32,        // output
    pub qpos_err_int: f32,  // error integration
    pub dt: f32,            // time step

    pub ulmt_int: f32,      // error integral upper limit
    pub llmt_int: f32,      // error integral lower limit

    pub ulmt_out: f32,      // output range upper limit
    pub llmt_out: f32,      // output range lower limit
}

impl PIDController {
    /// Creates a new PID Controller.
    pub fn new(kp: f32, ki: f32, kd: f32) -> PIDController {
        PIDController {
            kp: kp,
            ki: ki,
            kd: kd,
            qpos_dsr: 0.0,
            qvel_dsr: 0.0,
            qacc_dsr: 0.0,
            output: 0.0,
            qpos_err_int: 0.0,
            dt: 0.002,
            ulmt_int: f32::INFINITY,
            llmt_int: -f32::INFINITY,
            ulmt_out: f32::INFINITY,
            llmt_out: -f32::INFINITY
        }
    }

    pub fn set_limit(&mut self, ulmt_out: f32, llmt_out: f32, ulmt_int: f32, llmt_int: f32) {
        self.ulmt_out = ulmt_out;
        self.llmt_out = llmt_out;
        self.ulmt_int = ulmt_int;
        self.llmt_int = llmt_int;
    }

    pub fn set_time_step(&mut self, time_step: f32) {
        self.dt = time_step;
    }

    pub fn set_desired(&mut self, qacc: f32, qvel: f32, qpos: f32) {
        self.qacc_dsr = qacc;
        self.qvel_dsr = qvel;
        self.qpos_dsr = qpos;
    }

    pub fn set_current(&mut self, qvel: f32, qpos: f32) {
        let qpos_err = self.qpos_dsr - qpos;
        let qpos_err_int = self.qpos_err_int + self.dt * qpos_err;
        self.qpos_err_int = limit_range(self.llmt_int, self.ulmt_int, qpos_err_int);

        let output =
            self.kp * (self.qpos_dsr - qpos) +
            self.ki * (self.qpos_err_int) +
            self.kd * (self.qvel_dsr - qvel);

        self.output = limit_range(self.llmt_out, self.ulmt_out, output);
    }

    pub fn get_output(&self) -> f32 {
        return self.output;
    }

    pub fn reset(&mut self) {
        self.kp = 1.0;
        self.ki = 0.0;
        self.kd = 0.0;
        self.qpos_dsr = 0.0;
        self.qvel_dsr = 0.0;
        self.qacc_dsr = 0.0;
        self.qpos_err_int = 0.0;
        self.dt = 0.002;
        self.ulmt_int =  f32::INFINITY;
        self.llmt_int = -f32::INFINITY;
        self.ulmt_out =  f32::INFINITY;
        self.llmt_out = -f32::INFINITY;
    }
}