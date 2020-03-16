use crate::math::matrix::{Matrix3f, Vector3f};
use crate::utils::rotation::EulerAngle;

pub fn eul2rotm(eul: EulerAngle) -> Matrix3f {
    match eul {
        EulerAngle::ZYX(zyx) => {
            let cz = zyx[0].cos();
            let cy = zyx[1].cos();
            let cx = zyx[2].cos();
            let sz = zyx[0].sin();
            let sy = zyx[1].sin();
            let sx = zyx[2].sin();

            Matrix3f::new(
                cy*cz,   sy*sx*cz-sz*cx,    sy*cx*cz+sz*sx,
                cy*sz,   sy*sx*sz+cz*cx,    sy*cx*sz-cz*sx,
                  -sy,            cy*sx,             cy*cx,
            )
        },
        EulerAngle::XYZ(xyz) => {
            let cx = xyz[0].cos();
            let cy = xyz[1].cos();
            let cz = xyz[2].cos();
            let sx = xyz[0].sin();
            let sy = xyz[1].sin();
            let sz = xyz[2].sin();

            Matrix3f::new(
                           cy*cz,           -cy*sz,     sy,
                cx*sz + cz*sx*sy, cx*cz - sx*sy*sz, -cy*sx,
                sx*sz - cx*cz*sy, cz*sx + cx*sy*sz,  cx*cy,
            )
        },
        EulerAngle::ZYZ(zyz2) => {
            let cz  = zyz2[0].cos();
            let cy  = zyz2[1].cos();
            let cz2 = zyz2[2].cos();
            let sz  = zyz2[0].sin();
            let sy  = zyz2[1].sin();
            let sz2 = zyz2[2].sin();

            Matrix3f::new(
                cz2*cy*cz-sz2*sz,   -sz2*cy*cz-cz2*sz,    sy*cz,
                cz2*cy*sz+sz2*cz,   -sz2*cy*sz+cz2*cz,    sy*sz,
                         -cz2*sy,              sz2*sy,       cy,
            )
        },
    }
}