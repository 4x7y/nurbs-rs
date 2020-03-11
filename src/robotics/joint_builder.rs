use crate::robotics::*;
use crate::math::*;
use na::geometry::{Translation3, UnitQuaternion};

#[derive(Debug, Clone)]
pub struct JointBuilder {
    name: String,
    joint_type: JointType,
    limits: Option<Range>,
    origin: Isometry3f,
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
            limits: None,
            origin: Isometry3f::identity(),
        }
    }
    /// Set the name of the `Link`
    pub fn name(mut self, name: &str) -> JointBuilder {
        self.name = name.to_string();
        self
    }
    /// Set the tmp which is connected to this link
    pub fn joint_type(mut self, joint_type: JointType) -> JointBuilder {
        self.joint_type = joint_type;
        self
    }
    /// Set tmp limits
    pub fn limits(mut self, limits: Option<Range>) -> JointBuilder {
        self.limits = limits;
        self
    }
    /// Set the origin transform of this tmp
    pub fn origin(mut self, origin: Isometry3f) -> JointBuilder {
        self.origin = origin;
        self
    }
    /// Set the translation of the origin transform of this tmp
    pub fn translation(mut self, translation: Translation3<Scalar>) -> JointBuilder {
        self.origin.translation = translation;
        self
    }
    /// Set the rotation of the origin transform of this tmp
    pub fn rotation(mut self, rotation: UnitQuaternion<Scalar>) -> JointBuilder {
        self.origin.rotation = rotation;
        self
    }
    /// Create `Joint` instance
    pub fn finalize(self) -> Joint {
        let mut joint = Joint::new(&self.name, self.joint_type);
        joint.set_origin(self.origin);
        joint.limits = self.limits;
        joint
    }
    // Create `Node` instead of `Joint` as output
    // pub fn into_node(self) -> Node {
    //     self.finalize().into()
    // }
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