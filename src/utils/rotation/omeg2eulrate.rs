use crate::utils::rotation::EulerAngle;
use crate::math::Matrix3f;

/// Return the inverse of Euler angle rates matrix, $T^{-1}$. Transform angular velocity
/// to Euler angle rates through $\dot{\psi} = T^{-1} \omega$.
///
/// # Arguments
///
/// * `eulr` - Euler angles
/// * `seq` - Euler angle sequence
///
/// # Reference
///
/// * Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors
///   - Section 5.2: Euler Angle Rates and Angular Velocity
pub fn matx_omeg2eulr_dot(eulr: &EulerAngle) -> Matrix3f {
    return match eulr {
        // Eq. 457
        EulerAngle::ZYX(eulr) => {
            let z = eulr[0];
            let y = eulr[1];
            let cy = y.cos();
            let sy = y.sin();
            let cz = z.cos();
            let sz = z.sin();

            Matrix3f::new(cz * sy / cy, sy * sz / cy, 1.,
                          -sz,           cz, 0.,
                          cz / cy,      sz / cy, 0.)
        },
        EulerAngle::XYZ(eulr) => {
            let x = eulr[0];
            let y = eulr[1];
            let cx = x.cos();
            let sx = x.sin();
            let cy = y.cos();
            let sy = y.sin();

            Matrix3f::new(1., sx * sy / cy, -cx * sy / cy,
                          0.,           cx,            sx,
                          0.,     -sx / cy,       cx / cy)
        },
        EulerAngle::ZYZ(eulr) => {
            let z = eulr[0];
            let y = eulr[1];
            let cz = z.cos();
            let sz = z.sin();
            let cy = y.cos();
            let sy = y.sin();

            Matrix3f::new(-cy * cz / sy, -cy * sz / sy, 1.,
                                    -sz,            cz, 0.,
                                cz / sy,       sz / sy, 0.)
        },
    };
}