use crate::math::matrix::{Matrix3f, Vector3f};

pub enum EulerAngle {
    ZYX(Vector3f),
    XYZ(Vector3f),
    ZYZ(Vector3f),
}

/// Return Euler angle rates matrix $T$. Transform Euler angle rates to angular velocity
/// through $\omega = T \dot{\psi}$.
///
/// # Arguments
///
/// * `eulr` - Euler angles
/// * `seq` - Euler angle sequence
///
/// # Remarks
///
/// The time derivatives of the Euler angle vector is the vector of Euler angle rates.
/// The relationship between the Euler angle rates and the angular velocity of the body is
/// encoded in the Euler angle rates matrix.
///
/// Let $\hat{e}_i$ be the $i$-th unit vector, the function that maps an Euler angle
/// vector to its corresponding Euler angle rates matrix, $E \in \mathbb{R}^{3\times 3}$
/// , is
///
/// $$ E_{ijk}(x,y,z) = (R_k(z)^\top R_j(y)^\top \hat{e}_i, R_k(z)^\top \hat{e}_j, \hat{e}_k).$$
///
/// Hence
///
/// $$ \omega = E_{ijk}(\psi) \dot{\psi}. $$
///
/// # Reference
///
/// * Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors
///   - Section 5.2: Euler Angle Rates and Angular Velocity
pub fn matx_eulr_dot2omeg(eulr: &EulerAngle) -> Matrix3f {
    unimplemented!()
}


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