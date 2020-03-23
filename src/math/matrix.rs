use na::*;
use na::base::storage::Owned;
pub use na::{U1, U2, U3, U4, U5, U6};
use std::f64;

// scalar
pub type Scalar = f64;
pub(crate) const INFINITY: Scalar = f64::INFINITY;
pub(crate) const NEG_INFINITY: Scalar = f64::NEG_INFINITY;

// vector
pub type Vector2f = Vector2<Scalar>;
pub type Vector3f = Vector3<Scalar>;
pub type Vector4f = Vector4<Scalar>;
pub type Vector5f = Vector5<Scalar>;
pub type Vector6f = Vector6<Scalar>;
pub type VectorNf<D> = VectorN<Scalar, D>;
pub type VectorDf = DVector<Scalar>;

pub type Vector2f32 = Vector2<f32>;
pub type Vector3f32 = Vector3<f32>;
pub type Vector4f32 = Vector4<f32>;
pub type Vector5f32 = Vector5<f32>;
pub type Vector6f32 = Vector6<f32>;
pub type VectorNf32<D> = VectorN<f32, D>;
pub type VectorDf32 = DVector<f32>;

pub type VectorSlice2f<'a> = VectorSlice2<'a, Scalar>;
pub type VectorSlice3f<'a> = VectorSlice3<'a, Scalar>;
pub type VectorSlice4f<'a> = VectorSlice4<'a, Scalar>;
pub type VectorSlice5f<'a> = VectorSlice5<'a, Scalar>;
pub type VectorSlice6f<'a> = VectorSlice6<'a, Scalar>;

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
pub type Matrix3Df = Matrix<Scalar, U3, Dynamic, Owned<Scalar, U3, Dynamic>>;
pub type Matrix6Df = Matrix<Scalar, U6, Dynamic, Owned<Scalar, U6, Dynamic>>;

pub type Matrix3f32 = Matrix3<f32>;
pub type Matrix4f32 = Matrix4<f32>;
pub type Matrix6f32 = Matrix6<f32>;
pub type MatrixMNf32<R, C> = MatrixMN<f32, R, C>;
pub type MatrixDDf32 = DMatrix<f32>;
pub type Matrix6Df32 = Matrix<f32, U6, Dynamic, Owned<Scalar, U6, Dynamic>>;

// matrix slice
pub type MatrixSlice3f<'a> = MatrixSlice3<'a, Scalar>;



// screw
pub type Screw = Vector6f;


pub type Isometry3f = Isometry3<Scalar>;
pub type Isometry3f32 = Isometry3<f32>;

pub type UnitVector3f = Unit<Vector3f>;

pub type Translation3f32 = Translation3<f32>;
pub type Quaternionf32 = Quaternion<f32>;
pub type UnitQuat4f32 = UnitQuaternion<f32>;
pub type Rotation3f32 = Rotation3<f32>;