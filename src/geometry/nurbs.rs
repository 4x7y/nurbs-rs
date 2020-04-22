use crate::math::*;
use na::{Point3};
use std::cell::RefCell;
use std::rc::Rc;
use kiss3d::resource::Mesh;
use crate::geometry::{BSplineSurface, BoundingBox};

#[derive(Debug, Clone)]
pub struct NurbsSurface {
    pub order_u:        usize,
    pub order_v:        usize,
    pub degree_u:       usize,
    pub degree_v:       usize,
    pub knotvec_u:      Vec<Scalar>,
    pub knotvec_v:      Vec<Scalar>,
    pub ctrlpts_w:      Vec<Vector4f>,      // weighted control points in (x*w, y*w, z*w, w) format
    pub ctrlpts:        Vec<Vector3f>,      // control points in (x, y, z) format
    pub weights:        Vec<Scalar>,        // control point weights
    pub ctrlpts_size_u: usize,              // number of control points on the u-direction
    pub ctrlpts_size_v: usize,              // number of control points on the v-direction
    pub ctrlpts2d:      Vec<Vec<Vector4f>>, // 2-dimensional array of weighted control points
    pub delta:          Scalar,             // sample distance
    pub delta_u:        Scalar,             // sample distance on the u-direction
    pub delta_v:        Scalar,             // sample distance on the v-direction
    pub sample_size:    usize,
    pub sample_size_u:  u16,
    pub sample_size_v:  u16,
    pub bbox:           usize,
    pub name:           String,
    pub dimension:      usize,
    pub vis:            usize,
    pub rational:       bool,
    pub bs_surf:        BSplineSurface<U4>, // 4-dimensional b-spline surface
}

impl NurbsSurface {

    pub fn new(control_points: Vec<Vector3f>,
               knot_vector_u: Vec<Scalar>,
               knot_vector_v: Vec<Scalar>,
               degree_u: usize,
               degree_v: usize,
               size_u: usize,
               size_v: usize,
               weight: Vec<Scalar>) -> Self {

        let mut ctrlpts_w = Vec::<Vector4f>::new();
        for (point, w) in control_points.iter().zip(weight.clone()) {
            ctrlpts_w.push(Vector4f::new(point[0] * w, point[1] * w, point[2] * w, w))
        }

        NurbsSurface {
            order_u:  0,
            order_v:  0,
            degree_u: degree_u,
            degree_v: degree_v,
            knotvec_u: knot_vector_u.clone(),
            knotvec_v: knot_vector_v.clone(),
            ctrlpts_w: ctrlpts_w.clone(),
            ctrlpts: control_points.clone(),
            weights: weight.clone(),
            ctrlpts_size_u: size_u,
            ctrlpts_size_v: size_v,
            ctrlpts2d: vec![],
            delta: 0.0,
            delta_u: 0.0,
            delta_v: 0.0,
            sample_size: 0,
            sample_size_u: 100,
            sample_size_v: 100,
            bbox: 0,
            name: "".to_string(),
            dimension: 0,
            vis: 0,
            rational: true,
            bs_surf: BSplineSurface::new(ctrlpts_w, knot_vector_u, knot_vector_v, degree_u, degree_v, size_u, size_v),
        }
    }

    pub fn set_sample_size(&mut self, size_u: usize, size_v: usize) {
        self.sample_size_u = size_u as u16;
        self.sample_size_v = size_v as u16;
        self.sample_size = size_v * size_u;
    }

    pub fn get_mesh(&self) -> Rc<RefCell<Mesh>> {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let step_u = self.sample_size_u;
        let step_v = self.sample_size_v;

        for i in 0u16..step_u {
            for j in 0u16..step_v {
                let u = i as Scalar / step_u as Scalar;
                let v = j as Scalar / step_v as Scalar;
                let coord = self.bs_surf.evaluate_single(u, v);
                let w = coord[3] as f32;
                let x = coord[0] as f32 / w;
                let y = coord[1] as f32 / w;
                let z = coord[2] as f32 / w;
                vertices.push(Point3::new(x, y, z));

                if i > 0 && j > 0 {
                    let idx_1 = i * step_v + j;
                    let idx_2 = (i - 1) * step_v + j;
                    let idx_3 = i * step_v + (j - 1);
                    indices.push(Point3::new(idx_1, idx_2, idx_3))
                }

                if i < 99 && j < 99 {
                    let idx_1 = i * step_v + j;
                    let idx_2 = (i + 1) * step_v + j;
                    let idx_3 = i * step_v + (j + 1);
                    indices.push(Point3::new(idx_1, idx_2, idx_3))
                }
            }
        }

        let mesh = Rc::new(RefCell::new(Mesh::new(
            vertices, indices, None, None, false,
        )));

        return mesh;
    }
}