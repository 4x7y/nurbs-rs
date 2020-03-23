use crate::robotics::*;
use crate::math::*;
use na::geometry::{Translation3, UnitQuaternion};
use crate::utils::*;

#[derive(Debug, Clone)]
pub struct JointBuilder {
    name: String,
    joint_type: JointType,
    qpos_limit: Option<Range>,
    effort: Option<Scalar>,                   // joint effort
    dynamics: Option<JointDynamics>,          // joint dynamics coefficients (damping, friction)
    safe_ctrl: Option<JointSafetyController>, // joint safety controller
    tform_jnt2parent: Matrix4f,               // fixed transform from joint to parent frame
    tform_child2jnt: Matrix4f,                // fixed transform from child to joint frame
}

impl Default for JointBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl JointBuilder {
    pub fn new() -> JointBuilder {
        JointBuilder {
            name: "".to_string(),
            joint_type: JointType::Fixed,
            qpos_limit: None,
            effort: None,
            safe_ctrl: None,
            dynamics: None,
            tform_jnt2parent: Matrix4f::identity(),
            tform_child2jnt: Matrix4f::identity(),
        }
    }
    /// Set the name of the `Link`
    pub fn name(mut self, name: &str) -> Self {
        self.name = name.to_string();
        self
    }
    /// Set the tmp which is connected to this link
    pub fn joint_type(mut self, joint_type: JointType) -> Self {
        self.joint_type = joint_type;
        self
    }

    /// Set tmp limits
    pub fn limits(mut self, limits: Option<Range>) -> Self {
        self.qpos_limit = limits;
        self
    }
    // /// Set the origin transform of this tmp
    // pub fn origin(mut self, origin: Isometry3f) -> Self {
    //     self.origin = origin;
    //     self
    // }
    /// Set the translation of the origin transform of this tmp
    ///
    /// # Arguments
    /// - `xyz`: Represents the $$x,y,z$$ offset. All positions are
    ///          specified in meters.
    /// - `rpy`: Represents the rotation around fixed axis:
    ///          first roll around x, then pitch around y and finally
    ///          yaw around z. All angles are specified in radians.
    pub fn tform_jnt2parent(mut self, xyz: Vector3f, rpy: Vector3f) -> Self {
        let mut tform = Matrix4f::zeros();
        let ypr  = EulerAngle::ZYX(Vector3f::new(rpy[2], rpy[1], rpy[0]));
        let rotm = eul2rotm(ypr);
        tform.fixed_slice_mut::<U3, U3>(0, 0).copy_from(&rotm);
        tform.fixed_slice_mut::<U3, U1>(0, 3).copy_from(&xyz);
        tform[(3, 3)] = 1.;
        self.tform_jnt2parent = tform;

        self
    }

    pub fn safety_controller(mut self,
                             soft_lower_limit: Scalar,
                             soft_upper_limit: Scalar,
                             k_position: Scalar,
                             k_velocity: Scalar,) -> Self {
        self.safe_ctrl = Some(JointSafetyController {
            soft_lower_limit: soft_lower_limit,
            soft_upper_limit: soft_upper_limit,
            k_position: k_position,
            k_velocity: k_velocity,
        });
        self
    }

    pub fn dynamics(mut self, damping: Scalar, friction: Scalar, effort_limit: Scalar) -> Self {
        self.dynamics = Some(JointDynamics{
            damping: damping,
            friction: friction,
            effort_limit: effort_limit,
        });
        self
    }

    /// Create `Joint` instance
    pub fn finalize(self) -> Joint {
        let mut joint = Joint::new(&self.name, self.joint_type);
        joint.tform_jnt2parent = self.tform_jnt2parent;
        joint.qpos_limit = self.qpos_limit;
        joint.screw_axis = match &joint.joint_type {
            JointType::Prismatic { axis } =>
                Matrix6Df::from_row_slice(&[0., 0., 0., axis[0], axis[1], axis[2]]),
            JointType::Revolute { axis } =>
                Matrix6Df::from_row_slice(&[axis[0], axis[1], axis[2], 0., 0., 0.]),
            JointType::Fixed => {
                Matrix6Df::zeros(0)
            },
        };
        joint
    }
}

/// Information for copying tmp state of other tmp
///
/// For example, `Mimic` is used to calculate the position of the
/// gripper(R) from gripper(L). In that case, the code like below
/// will be used.
///
/// Output position (mimic_position() is calculated by
///     `qpos = tmp[name] * multiplier + origin`
///
#[derive(Debug, Clone)]
pub struct Mimic {
    pub multiplier: Scalar,
    pub origin: Scalar,
}

impl Mimic {

    pub fn new(multiplier: Scalar, origin: Scalar) -> Self {
        Mimic { multiplier, origin }
    }

    pub fn mimic_position(&self, from_position: Scalar) -> Scalar {
        from_position * self.multiplier + self.origin
    }
}