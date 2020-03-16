use crate::math::matrix::{Matrix3f, Vector3f};
use crate::utils::*;
use crate::robotics::near_zero;

/// Calculates the Euler angles corresponding to the input
/// rotation matrix, `rotm`. The Euler angles follow the
/// body-fixed (intrinsic) axis rotation order specified
/// in `seq`.
pub fn rotm2eul(rotm: Matrix3f, seq: EulerAngleOrder) -> EulerAngle {

    match seq {
        EulerAngleOrder::ZYX => {
            let sy = (rotm[(0, 0)].powi(2) + rotm[(1, 0)].powi(2)).sqrt();
            if near_zero(sy) {
                EulerAngle::ZYX(Vector3f::new(
                    0.,
                    (-rotm[(2, 0)]).atan2(sy),
                    (-rotm[(1, 2)]).atan2(rotm[(1, 1)]),
                ))
            } else {
                EulerAngle::ZYX(Vector3f::new(
                    rotm[(1, 0)].atan2(rotm[(0, 0)]),
                    (-rotm[(2, 0)]).atan2(sy),
                    rotm[(2, 1)].atan2(rotm[(2, 2)]),
                ))
            }
        },
        EulerAngleOrder::XYZ => {
            unimplemented!();
            // EulerAngle::XYZ(Vector3f::new(
            //     rotm[(1, 2)].atan2(rotm[(2, 2)]),
            //     rotm[(0, 2)].asin(),
            //     rotm[(0, 1)].atan2(rotm[(0, 0)]),
            // ))
        },
        EulerAngleOrder::ZYZ => {
            unimplemented!();
            // EulerAngle::ZYZ(Vector3f::new(
            //     rotm[(1, 2)].atan2(- rotm[(0, 2)]),
            //     rotm[(2, 2)].acos(),
            //     rotm[(2, 1)].atan2(rotm[(2, 0)]),
            // ))
        },
    }
}