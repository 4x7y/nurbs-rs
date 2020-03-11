use crate::math::*;


//
///// Converts a spatial velocity vector into a 4x4 matrix in se3
//pub fn vec_to_se3(vec: &Vector6f) -> Matrix4f {
//    let vec = vec.fixed_rows::<U3>(0);
//    let so3 = vec_to_so3(&Vector3f::from(vec));
//    Matrix4f::new(
//        so3[(0, 0)], so3[(0, 1)], so3[(0, 2)], vec[4],
//        so3[(1, 0)], so3[(1, 1)], so3[(1, 2)], vec[5],
//        so3[(2, 0)], so3[(2, 1)], so3[(2, 2)], vec[6],
//                0.0,         0.0,         0.0,    0.0,
//    )
//}

/// Converts a 3-vector of exponential coordinates for rotation into
/// axis-angle form
pub fn eulr_vec_to_axis_angle(eulr_vec: &Vector3f) -> (Vector3f, Scalar) {
    return (eulr_vec.normalize(), eulr_vec.norm());
}

///// Computes the matrix exponential of an se3 representation of
///// exponential coordinates
//pub fn exp_mat6x6(se3mat: &Matrix6f) -> f32 {
//    let so3mat = se3mat.slice((0, 0), (3, 3));
//    let omeg = so3_to_vec(&so3mat);
//
//    // Todo: near zero
//
//
//
//    return 0.;
//}

//pub fn adjoint(trans: &Matrix4f) -> Matrix6f {
//    return Matrix6f::identity();
//}

///// Computes the space Jacobian for an open chain robot
//pub fn jacb(screws: &Matrix6Nf, qpos: VectorNf) -> Matrix6Nf {
//    let mut jacb = Matrix6Nf::zeros(qpos.len());
//    let mut mat4_trans = Matrix4f::identity();
//    for i in 0..qpos.len() {
//        mat4_trans = mat4_trans;
//        jacb.set_column(i, &Vector6f::zeros());
//    }
//
//    return jacb;
//}