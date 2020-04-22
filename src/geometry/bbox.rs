use crate::math::*;
use crate::geometry::NurbsSurface;
use core::fmt;
use failure::_core::fmt::Formatter;
use kiss3d::scene::SceneNode;
use crate::simulation::sim_model::SimScene;

#[derive(Clone)]
pub struct BoundingBox {
    pub rotm: Matrix3f,
    pub min: Vector3f,
    pub max: Vector3f,
    pub scene_node: Option<SceneNode>,
}

impl BoundingBox {

    fn center(points: &Vec<Vector3f>) -> Vector3f {
        assert!(points.len() > 0);

        let mut center = Vector3f::zeros();
        for ctrlpt in points {
            center += ctrlpt;
        }
        return center / points.len() as Scalar;
    }

    fn cov(points: &Vec<Vector3f>) -> Matrix3f {
        let mut cov = Matrix3f::zeros();
        let mu = Self::center(points);

        for i in 0..3 {
            for j in 0..3 {
                for point in points {
                    let diff = point - mu;
                    cov[(i, j)] += diff[i] * diff[j];
                }
            }
        }

        return cov;
    }

    pub fn from_point_cloud(point_cloud: &Vec<Vector3f>) -> Self {
        assert!(point_cloud.len() > 0);

        let cov = Self::cov(point_cloud);
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

        BoundingBox {
            rotm: rotm_b2w,
            min: min,
            max: max,
            scene_node: None,
        }
    }

    pub fn register_scene(&mut self, scene: &mut SimScene) {
        let wx = (self.max[0] - self.min[0]) as f32;
        let wy = (self.max[1] - self.min[1]) as f32;
        let wz = (self.max[2] - self.min[2]) as f32;

        let mut h = scene.window.add_cube(wx, wy, wz);
        h.set_color(0., 0.5, 0.);
        self.scene_node = Some(h);
    }

    pub fn render(&mut self) {
        if let Some(node) = &mut self.scene_node {
            let tvec = &(self.max + &self.min) / 2.;
            let tvec = self.rotm * tvec;
            let tvec = Translation3f32::new(
                tvec[0] as f32, tvec[1] as f32, tvec[2] as f32);
            let rotm = &self.rotm;
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

impl <'a> From<&'a NurbsSurface> for BoundingBox {
    fn from(surf: &'a NurbsSurface) -> Self {
        Self::from_point_cloud(&surf.ctrlpts)
    }
}

impl fmt::Display for BoundingBox {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.min, self.max)
    }
}