use crate::math::*;

pub fn rotm2quat(rotm: Matrix3f) -> Vector4f {
    let tr = rotm[(0, 0)] + rotm[(1, 1)] + rotm[(2, 2)];
    let qw;
    let qx;
    let qy;
    let qz;
    
    if tr > 0. {
        let s = (tr + 1.).sqrt() * 2.; // S=4*qw
        qw = 0.25 * s;
        qx = (rotm[(2, 1)] - rotm[(1, 2)]) / s;
        qy = (rotm[(0, 2)] - rotm[(2, 0)]) / s;
        qz = (rotm[(1, 0)] - rotm[(0, 1)]) / s;
    } else if (rotm[(0, 0)] > rotm[(1, 1)]) & (rotm[(0, 0)] > rotm[(2, 2)]) {
        let s = (1.0 + rotm[(0, 0)] - rotm[(1, 1)] - rotm[(2, 2)]).sqrt() * 2.; // S=4*qx
        qw = (rotm[(2, 1)] - rotm[(1, 2)]) / s;
        qx = 0.25 * s;
        qy = (rotm[(0, 1)] + rotm[(1, 0)]) / s;
        qz = (rotm[(0, 2)] + rotm[(2, 0)]) / s;
    } else if rotm[(1, 1)] > rotm[(2, 2)] {
        let s = (1.0 + rotm[(1, 1)] - rotm[(0, 0)] - rotm[(2, 2)]).sqrt() * 2.; // S=4*qy
        qw = (rotm[(0, 2)] - rotm[(2, 0)]) / s;
        qx = (rotm[(0, 1)] + rotm[(1, 0)]) / s;
        qy = 0.25 * s;
        qz = (rotm[(1, 2)] + rotm[(2, 1)]) / s;
    } else {
        let s = (1.0 + rotm[(2, 2)] - rotm[(0, 0)] - rotm[(1, 1)]).sqrt() * 2.; // S=4*qz
        qw = (rotm[(1, 0)] - rotm[(0, 1)]) / s;
        qx = (rotm[(0, 2)] + rotm[(2, 0)]) / s;
        qy = (rotm[(1, 2)] + rotm[(2, 1)]) / s;
        qz = 0.25 * s;
    }
    
    return Vector4f::new(qw, qx, qy, qz);
}