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

pub fn quat2rotm(quat: &Vector4f) -> Matrix3f {

    let quat_normalized = quat.normalize();
    let s = quat_normalized[0];
    let x = quat_normalized[1];
    let y = quat_normalized[2];
    let z = quat_normalized[3];
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let sx = s * x;
    let sy = s * y;
    let sz = s * z;
    
    let rotm = Matrix3f::new(
        1. - 2. * (yy+zz),      2. * (xy-sz),      2. * (xz+sy),
             2. * (xy+sz), 1. - 2. * (xx+zz),      2. * (yz-sx),
             2. * (xz-sy),      2. * (yz+sx), 1. - 2. * (xx+yy),
    );

    return rotm;
}