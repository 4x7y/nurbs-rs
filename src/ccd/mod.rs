pub mod obb;
pub mod obb_tree;
mod mpr;
mod helper;
mod object;

pub use self::obb::*;
pub use self::obb_tree::*;
pub use self::mpr::*;
pub use self::helper::*;
pub use self::object::*;
use crate::math::{Vector3f, Scalar};


pub struct CCDCriteria {
    pub max_iterations: usize,
    pub epa_tolerance:  Scalar,
    pub mpr_tolerance:  Scalar,
    pub dist_tolerance: Scalar,
}

pub trait CCDObject {

    /// returns center or any point near center
    fn center(&self) -> Vector3f;

    /// returns furthest point from object (shape) in specified direction.
    fn support(&self, dir: &Vector3f) -> Vector3f;
}


impl CCDCriteria {

    pub fn default() -> Self {
        CCDCriteria {
            max_iterations: 1000,
            epa_tolerance:  1e-4,
            mpr_tolerance:  1e-4,
            dist_tolerance: 1e-6,
        }
    }
}