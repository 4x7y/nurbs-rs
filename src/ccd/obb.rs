use crate::math::*;
use crate::simulation::sim_model::SimScene;
use kiss3d::scene::SceneNode;
use crate::geometry::NurbsSurface;
use core::fmt;
use std::fmt::Formatter;

pub struct OBB {
    pub pos:  Vector3f,      // The center position of this OBB.
    pub r:    Vector3f,      // Half-sizes to x, y and z directions in the local space.
    pub axis: Matrix3f,      // Normalized direction vectors for the local axes.
    pub scene_node: Option<SceneNode>,
}

impl OBB {
    pub fn intersects(&self, b: &OBB, epsilon: Scalar) -> bool {
        // Generate a rotation matrix that transforms from world
        // space to this OBB's coordinate space.
        let rotm_b2a = self.axis.transpose() * &b.axis;
        // for i in 0..3 {
        //     for j in 0..3 {
        //         R[(i, j)] = self.axis[i].dot(&b.axis[j]);
        //     }
        // }

        let t: Vector3f = b.pos - self.pos;
        // Express the translation vector in a's coordinate frame.
        // let t = Vector3f::new(
        //     t.dot(&self.axis[0]), t.dot(&self.axis[1]), t.dot(&self.axis[2])
        // );
        let t = self.axis.transpose() * t;

        let mut AbsR = Matrix3f::zeros();
        for i in 0..3 {
            for j in 0..3 {
                AbsR[(i, j)] = rotm_b2a[(i, j)].abs() + epsilon;
            }
        }

        // Test the three major axes of this OBB.
        for i in 0..3 {
            let ra = self.r[i];
            let rb = b.r[0] * AbsR[(i, 0)] + b.r[1] * AbsR[(i, 1)] + b.r[2] * AbsR[(i, 2)];
            if t[i].abs() > ra + rb {
                return false;
            }
        }

        // Test the three major axes of the OBB b.
        for i in 0..3 {
            let ra = 
                self.r[0] * AbsR[(0, i)] +
                self.r[1] * AbsR[(1, i)] +
                self.r[2] * AbsR[(2, i)];
            let rb = b.r[i];
            if (t[0] * rotm_b2a[(0, i)] + t[1] * rotm_b2a[(1, i)] + t[2] * rotm_b2a[(2, i)]).abs() > ra + rb {
                return false;
            }
        }

        // Test the 9 different cross-axes.

        // A.x <cross> B.x
        let ra = self.r.y * AbsR[(2, 0)] + self.r[2] * AbsR[(1, 0)];
        let rb = b.r[1] * AbsR[(0, 2)] + b.r[2] * AbsR[(0, 1)];
        if (t[2] * rotm_b2a[(1, 0)] - t[1] * rotm_b2a[(2, 0)]).abs() > ra + rb {
            return false;
        }

        // A.x < cross> B.y
        let ra = self.r[1] * AbsR[(2, 1)] + self.r[2] * AbsR[(1, 1)];
        let rb = b.r[0] * AbsR[(0, 2)] + b.r[2] * AbsR[(0, 0)];
        if (t[2] * rotm_b2a[(1, 1)] - t[1] * rotm_b2a[(2, 1)]).abs() > ra + rb {
            return false;
        }

        // A.x <cross> B.z
        let ra = self.r[1] * AbsR[(2, 2)] + self.r[2] * AbsR[(1, 2)];
        let rb = b.r[0] * AbsR[(0, 1)] + b.r[1] * AbsR[(0, 0)];
        if (t[2] * rotm_b2a[(1, 2)] - t[1] * rotm_b2a[(2, 2)]).abs() > ra + rb {
            return false;
        }

        // A.y <cross> B.x
        let ra = self.r[0] * AbsR[(2, 0)] + self.r[2] * AbsR[(0, 0)];
        let rb = b.r[1] * AbsR[(1, 2)] + b.r[2] * AbsR[(1, 1)];
        if (t[0] * rotm_b2a[(2, 0)] - t[2] * rotm_b2a[(0, 0)]).abs() > ra + rb {
            return false;
        }

        // A.y <cross> B.y
        let ra = self.r[0] * AbsR[(2, 1)] + self.r[2] * AbsR[(0, 1)];
        let rb = b.r[0] * AbsR[(1, 2)] + b.r[2] * AbsR[(1, 0)];
        if (t[0] * rotm_b2a[(2, 1)] - t[2] * rotm_b2a[(0, 1)]).abs() > ra + rb {
            return false;
        }

        // A.y <cross> B.z
        let ra = self.r[0] * AbsR[(2, 2)] + self.r[2] * AbsR[(0, 2)];
        let rb = b.r[0] * AbsR[(1, 1)] + b.r[1] * AbsR[(1, 0)];
        if (t[0] * rotm_b2a[(2, 2)] - t[2] * rotm_b2a[(0, 2)]).abs() > ra + rb {
            return false;
        }

        // A.z <cross> B.x
        let ra = self.r[0] * AbsR[(1, 0)] + self.r[1] * AbsR[(0, 0)];
        let rb = b.r[1] * AbsR[(2, 2)] + b.r[2] * AbsR[(2, 1)];
        if (t[1] * rotm_b2a[(0, 0)] - t[0] * rotm_b2a[(1, 0)]).abs() > ra + rb {
            return false;
        }

        // A.z <cross> B.y
        let ra = self.r[0] * AbsR[(1, 1)] + self.r[1] * AbsR[(0, 1)];
        let rb = b.r[0] * AbsR[(2, 2)] + b.r[2] * AbsR[(2, 0)];
        if (t[1] * rotm_b2a[(0, 1)] - t[0] * rotm_b2a[(1, 1)]).abs() > ra + rb {
            return false;
        }

        // A.z <cross> B.z
        let ra = self.r[0] * AbsR[(1, 2)] + self.r[1] * AbsR[(0, 2)];
        let rb = b.r[0] * AbsR[(2, 1)] + b.r[1] * AbsR[(2, 0)];
        if (t[1] * rotm_b2a[(0, 2)] - t[0] * rotm_b2a[(1, 2)]).abs() > ra + rb {
            return false;
        }

        // No separating axis exists, so the two OBB don't intersect.
        return true;
    }

    fn center(points: &Vec<Vector3f>) -> Vector3f {
        assert!(points.len() > 0);

        let mut center = Vector3f::zeros();
        for ctrlpt in points {
            center += ctrlpt;
        }
        return center / points.len() as Scalar;
    }

    fn cov(points: &Vec<Vector3f>, center: &Vector3f) -> Matrix3f {
        let mut cov = Matrix3f::zeros();

        for i in 0..3 {
            for j in 0..3 {
                for point in points {
                    let diff = point - center;
                    cov[(i, j)] += diff[i] * diff[j];
                }
            }
        }

        return cov;
    }

    pub fn from_point_cloud(point_cloud: &Vec<Vector3f>) -> Self {
        assert!(point_cloud.len() > 0);

        let center = Self::center(point_cloud);
        let cov = Self::cov(point_cloud, &center);
        let schur = cov.schur();
        let (mut rotm_b2w, vals) = schur.unpack(); // eigen vectors, eigen values
        if rotm_b2w.determinant() < 0. {
            rotm_b2w[(0, 0)] = -rotm_b2w[(0, 0)];
            rotm_b2w[(1, 0)] = -rotm_b2w[(1, 0)];
            rotm_b2w[(2, 0)] = -rotm_b2w[(2, 0)];
        }
        let rotm_w2b = rotm_b2w.transpose();

        let mut points_local = Vec::new();
        for point in point_cloud {
            points_local.push(rotm_w2b * point);
        }

        let mut min = points_local[0];
        let mut max = points_local[0];

        for point in points_local {
            max = na::sup(&max, &point);
            min = na::inf(&min, &point);
        }

        let pos = rotm_b2w * (max + min) / 2.;

        OBB {
            pos: pos,
            r: (max - min) / 2.,
            axis: rotm_b2w,
            scene_node: None
        }
    }

    pub fn register_scene(&mut self, scene: &mut SimScene) {
        let wx = self.r[0] as f32 * 2.;
        let wy = self.r[1] as f32 * 2.;
        let wz = self.r[2] as f32 * 2.;

        let mut h = scene.window.add_cube(wx, wy, wz);
        h.set_color(0., 0.5, 0.);
        self.scene_node = Some(h);
    }

    pub fn render(&mut self) {
        if let Some(node) = &mut self.scene_node {
            let tvec = self.pos;
            let tvec = Translation3f32::new(
                tvec[0] as f32, tvec[1] as f32, tvec[2] as f32);
            let rotm = &self.axis;
            let rotm = Rotation3f32::from_matrix_unchecked(Matrix3f32::new(
                rotm[(0, 0)] as f32, rotm[(0, 1)] as f32, rotm[(0, 2)] as f32,
                rotm[(1, 0)] as f32, rotm[(1, 1)] as f32, rotm[(1, 2)] as f32,
                rotm[(2, 0)] as f32, rotm[(2, 1)] as f32, rotm[(2, 2)] as f32,
            ));
            let quat = UnitQuat4f32::from_rotation_matrix(&rotm);
            let isometry = Isometry3f32::from_parts(tvec, quat);
            node.set_local_transformation(isometry);
        }
    }
}

impl <'a> From<&'a NurbsSurface> for OBB {
    fn from(surf: &'a NurbsSurface) -> Self {
        Self::from_point_cloud(&surf.ctrlpts)
    }
}

impl fmt::Display for OBB {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.axis, self.r)
    }
}

