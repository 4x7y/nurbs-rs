use na::*;
use na::base::storage::Owned;

// scalar
pub type Scalar = f32;

// vector
pub type Vector2f = Vector2<Scalar>;
pub type Vector3f = Vector3<Scalar>;
pub type Vector4f = Vector4<Scalar>;
pub type Vector5f = Vector5<Scalar>;
pub type Vector6f = Vector6<Scalar>;
pub type VectorNf<D> = VectorN<Scalar, D>;
pub type VectorDf = DVector<Scalar>;

// point
pub type Point2f = Point2<Scalar>;
pub type Point3f = Point3<Scalar>;
pub type Point4f = Point4<Scalar>;

// matrix
pub type Matrix3f = Matrix3<Scalar>;
pub type Matrix4f = Matrix4<Scalar>;
pub type Matrix6f = Matrix6<Scalar>;
pub type MatrixMNf<R, C> = MatrixMN<Scalar, R, C>;
pub type MatrixDDf = DMatrix<Scalar>;
pub type Matrix6Df = Matrix<Scalar, U6, Dynamic, Owned<Scalar, U6, Dynamic>>;

// screw
pub type Screw = Vector6f;

/// Converts a 3-vector to an so(3) representation
pub fn vec_to_so3(omeg: &Vector3f) -> Matrix3f {
    Matrix3f::new(
             0.0, -omeg[2],  omeg[1],
         omeg[2],      0.0, -omeg[0],
        -omeg[1],  omeg[0],      0.0,
    )
}

/// Converts an so(3) representation to a 3-vector
pub fn so3_to_vec(so3mat: &Matrix3f) -> Vector3f {
    Vector3f::new(
        so3mat[(2, 1)],
        so3mat[(0, 2)],
        so3mat[(1, 0)])
}
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