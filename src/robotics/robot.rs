use crate::math::vector::*;

pub struct Robot {
    pub nq: usize,                    // dimension of generalized coordinates
    pub nv: usize,                    // dimension of generalized velocities
    pub nbody: usize,                 // number of bodies

    // state
    pub qpos: VectorNf,               // generalized coordinates position         (nq x 1)
    pub qvel: VectorNf,               // generalized coordinates velocity         (nv x 1)

    // control
    pub ctrl: VectorNf,               // control input to the actuators           (nu x 1)
    pub qfrc_applied: VectorNf,       // applied generalized force                (nv x 1)
    pub xfrc_applied: VectorNf,       // applied Cartesian force/torque           (nbody x 6)

    // dynamics
    pub qacc: VectorNf,               // acceleration                             (nv x 1)
    pub act_dot: VectorNf,            // time-derivative of actuator activation   (na x 1)
    pub mm: MatrixMNf,                // total inertia                            (nv x nv)
    pub crb: VectorNf,                // com-based composite inertia and mass     (nbody x 10）
}

// type of sensor
pub enum SensorType {
    // common robotic sensors, attached to a site
    Touch,                   // scalar contact normal forces summed over sensor zone
    Accelerometer,           // 3D linear acceleration, in local frame
    Velocimeter,             // 3D linear velocity, in local frame
    Gyro,                    // 3D angular velocity, in local frame
    Force,                   // 3D force between site's body and its parent body
    Torque,                  // 3D torque between site's body and its parent body

    // sensors related to scalar joints, tendons, actuators
    JointPos,                // scalar joint position (hinge and slide only)
    JointVel,                // scalar joint velocity (hinge and slide only)
    TendonPos,               // scalar tendon position
    TendonVel,               // scalar tendon velocity
    ActuatorPos,             // scalar actuator position
    ActuatorVel,             // scalar actuator velocity
    ActuatorFrc,             // scalar actuator force

    // sensors related to ball joints
    BallQuat,                // 4D ball joint quaternion
    BallAngleVel,            // 3D ball joint angular velocity

    // joint and tendon limit sensors, in constraint space
    JOINTLIMITPOS,           // joint limit distance-margin
    JOINTLIMITVEL,           // joint limit velocity
    JOINTLIMITFRC,           // joint limit force
    TENDONLIMITPOS,          // tendon limit distance-margin
    TENDONLIMITVEL,          // tendon limit velocity
    TENDONLIMITFRC,          // tendon limit force

    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    FRAMEPOS,                // 3D position
    FRAMEQUAT,               // 4D unit quaternion orientation
    FRAMEXAXIS,              // 3D unit vector: x-axis of object's frame
    FRAMEYAXIS,              // 3D unit vector: y-axis of object's frame
    FRAMEZAXIS,              // 3D unit vector: z-axis of object's frame
    FRAMELINVEL,             // 3D linear velocity
    FRAMEANGVEL,             // 3D angular velocity
    FRAMELINACC,             // 3D linear acceleration
    FRAMEANGACC,             // 3D angular acceleration

    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    SUBTREECOM,              // 3D center of mass of subtree
    SUBTREELINVEL,           // 3D linear velocity of subtree
    SUBTREEANGMOM,           // 3D angular momentum of subtree
}