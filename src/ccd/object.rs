use crate::math::{Vector3f, Matrix3f, Scalar};
use crate::ccd::{CCDObject, CCD_ZERO, is_zero_approx, CCD_EPS};

pub struct Box {
    pub pos: Vector3f,
    pub rotm: Matrix3f,
    pub dim: Vector3f,
}

impl CCDObject for Box {
    fn center(&self) -> Vector3f {
        self.pos.clone_owned()
    }

    fn support(&self, dir: &Vector3f) -> Vector3f {
        let dir_local = self.rotm.transpose() * dir;
        // compute support point in specified direction
        let vec_local = Vector3f::new(
            dir_local[0].signum() * self.dim[0] * 0.5,
            dir_local[1].signum() * self.dim[1] * 0.5,
            dir_local[2].signum() * self.dim[2] * 0.5
        );
        let vec = self.rotm * &vec_local + &self.pos;
        return vec;
    }
}

pub struct Sphere {
    pub pos: Vector3f,
    pub rotm: Matrix3f,
    pub radius: Scalar,
}

impl CCDObject for Sphere {
    fn center(&self) -> Vector3f {
        self.pos.clone_owned()
    }

    fn support(&self, dir: &Vector3f) -> Vector3f {
        let dir_local = self.rotm.transpose() * dir;
        let len = dir_local.norm_squared();
        let vec_local = if len - CCD_EPS > CCD_ZERO {
            dir * self.radius / len.sqrt()
        } else {
            Vector3f::zeros()
        };
        let vec = self.rotm * &vec_local + &self.pos;
        return vec;
    }
}

pub struct Cylinder {
    pub pos: Vector3f,
    pub rotm: Matrix3f,
    pub radius: Scalar,
    pub height: Scalar,
}

impl CCDObject for Cylinder {
    fn center(&self) -> Vector3f {
        self.pos.clone_owned()
    }

    fn support(&self, dir: &Vector3f) -> Vector3f {
        let dir_local = self.rotm.transpose() * dir;

        let dist_z = (dir_local[0] * dir_local[0] + dir_local[1] * dir_local[1]).sqrt();
        let vec_local = if is_zero_approx(dist_z) {
            Vector3f::new(CCD_ZERO,
                          CCD_ZERO,
                          dir_local[2].signum()  * self.height * 0.5)
        } else {
            let rad = self.radius / dist_z;
            Vector3f::new(rad * dir_local[0],
                          rad * dir_local[1],
                          dir_local[2].signum() * self.height * 0.5)
        };

        let vec = self.rotm * &vec_local + &self.pos;
        return vec;
    }
}
