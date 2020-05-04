pub mod obb;
pub mod obb_tree;
mod mpr;
mod helper;

pub use self::obb::*;
pub use self::obb_tree::*;
pub use self::mpr::*;
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

//
// impl CCD {
//
//     pub fn new() -> Self {
//         CCD {
//             first_dir: (),
//             center1: (),
//             center2: (),
//             support1: (),
//             support2: (),
//             max_iterations: 0,
//             epa_tolerance: (),
//             mpr_tolerance: (),
//             dist_tolerance: ()
//         }
//     }
// }