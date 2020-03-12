use crate::robotics::kinematics::*;
use crate::math::*;

#[test]
fn test_trans2rp() {
    let trans = Matrix4f::new(
        1., 0.,  0., 0.,
        0., 0., -1., 0.,
        0., 1.,  0., 3.,
        0., 0.,  0., 1.);
    let rp = trans_to_rp(trans);
    // println!("{}", rp.0);
    // println!("{}", rp.1);
    assert_eq!(rp.0, Matrix3f::new(
        1., 0.,  0.,
        0., 0., -1.,
        0., 1.,  0.,
    ));
    assert_eq!(rp.1, Vector3f::new(0., 0., 3.));
}

#[test]
fn test_vec2so3() {
    let omeg = Vector3f::new(1., 2., 3.);
    let so3 = vec_to_so3(omeg);
    assert_eq!(so3, Matrix3f::new(
         0., -3.,  2.,
         3.,  0., -1.,
        -2.,  1.,  0.,
    ));
}

#[test]
fn test_adjoint() {
    let trans = Matrix4f::new(
        1., 0.,  0., 0.,
        0., 0., -1., 0.,
        0., 1.,  0., 3.,
        0., 0.,  0., 1.,);
    let adt = adjoint(trans);
    assert_eq!(adt, Matrix6f::new(
        1., 0., 0., 0., 0., 0.,
        0., 0.,-1., 0., 0., 0.,
        0., 1., 0., 0., 0., 0.,
        0., 0., 3., 1., 0., 0.,
        3., 0., 0., 0., 0.,-1.,
        0., 0., 0., 0., 1., 0.,
    ));
}

#[test]
fn test_trans_inv() {
    let trans = Matrix4f::new(
        1., 0.,  0., 0.,
        0., 0., -1., 0.,
        0., 1.,  0., 3.,
        0., 0.,  0., 1.,);
    let trans_inv = trans_inv(trans);
    assert_eq!(trans_inv, Matrix4f::new(
        1., 0., 0., 0.,
        0., 0., 1.,-3.,
        0.,-1., 0., 0.,
        0., 0., 0., 1.,
    ));
}

#[test]
fn test_vec2se3() {
    let velc = Vector6f::new(1., 2., 3., 4., 5., 6.);
    let se3 = vec_to_se3(velc);
    assert_eq!(se3, Matrix4f::new(
        0.,-3., 2., 4.,
        3., 0., -1.,5.,
       -2., 1., 0., 6.,
        0., 0., 0., 0.,
    ));
}

#[test]
fn test_axis_ang() {
    let res = axis_angle(Vector3f::new(1., 2., 3.));
    assert_relative_eq!(res.0, Vector3f::new(0.26726124, 0.53452248, 0.80178373), epsilon=1e-6);
    assert_relative_eq!(res.1, 3.7416573867739413, epsilon=1e-6);
}

#[test]
fn test_so3_to_vec() {
    let so3mat = Matrix3f::new(
         0., -3.,  2.,
         3.,  0., -1.,
        -2.,  1.,  0.,
    );
    let vec = so3_to_vec(so3mat);
    assert_relative_eq!(vec, Vector3f::new(1., 2., 3.), epsilon=1e-6);
}

#[test]
fn test_matrix_exp3() {
    let so3mat = Matrix3f::new(
        0., -3.,  2.,
        3.,  0., -1.,
       -2.,  1.,  0.,
    );
    let tform = matrix_exp3(so3mat);
    assert_relative_eq!(tform, Matrix3f::new(
        -0.69492056,  0.71352099,  0.08929286,
        -0.19200697, -0.30378504,  0.93319235,
         0.69297817,  0.6313497 ,  0.34810748,
    ), epsilon=1e-6);
}

#[test]
fn test_matrix_exp6() {
    let a = 1.57079632;
    let b = 2.35619449;
    let se3mat = Matrix4f::new(
        0., 0., 0., 0.,
        0., 0., -a,  b,
        0.,  a, 0.,  b,
        0., 0., 0., 0.,
    );
    let tform = matrix_exp6(se3mat);
    assert_relative_eq!(tform, Matrix4f::new(
        1., 0., 0., 0.,
        0., 0.,-1., 0.,
        0., 1., 0., 3.,
        0., 0., 0., 1.,
    ), epsilon=1e-6);
}

#[test]
fn test_near_zero_smaller() {
    assert_eq!(true, near_zero(1.0e-7));
}

#[test]
fn test_near_zero_greater() {
    assert_eq!(false, near_zero(1.0e-5));
}