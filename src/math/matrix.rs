use na::*;
use na::base::storage::Owned;
pub use na::{U1, U2, U3, U4, U5, U6};

// scalar
pub type Scalar = f64;

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

// matrix slice
pub type MatrixSlice3f<'a> = MatrixSlice3<'a, Scalar>;

// screw
pub type Screw = Vector6f;