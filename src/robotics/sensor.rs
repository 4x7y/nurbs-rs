
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
    JointLimitPos,           // joint limit distance-margin
    JointLimitVel,           // joint limit velocity
    JointLimitFrc,           // joint limit force
    TendonLimitPos,          // tendon limit distance-margin
    TendonLimitVel,          // tendon limit velocity
    TendonLimitFrc,          // tendon limit force

    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    FramePos,                // 3D position
    FrameQuat,               // 4D unit quaternion orientation
    FrameXAxis,              // 3D unit vector: x-axis of object's frame
    FrameYAxis,              // 3D unit vector: y-axis of object's frame
    FrameZAxis,              // 3D unit vector: z-axis of object's frame
    FrameLinVel,             // 3D linear velocity
    FrameAngVel,             // 3D angular velocity
    FrameLinAcc,             // 3D linear acceleration
    FrameAngAcc,             // 3D angular acceleration

    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    SubTreeCom,              // 3D center of mass of subtree
    SubTreeLinVel,           // 3D linear velocity of subtree
    SubTreeAngMom,           // 3D angular momentum of subtree
}