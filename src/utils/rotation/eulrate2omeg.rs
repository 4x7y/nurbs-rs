use crate::utils::rotation::EulerAngle;
use crate::math::Matrix3f;

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

