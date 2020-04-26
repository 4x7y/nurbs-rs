use crate::math::*;
use na::{Point3};
use std::cell::RefCell;
use std::rc::Rc;
use kiss3d::resource::Mesh;
use crate::geometry::{BSplineSurface, homo2cart};
use crate::geometry::helper;

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
    pub bspline:        BSplineSurface<U4>, // 4-dimensional b-spline surface
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

        let knot_vector_u = helper::normalize_knotvec(&knot_vector_u);
        let knot_vector_v = helper::normalize_knotvec(&knot_vector_v);

        NurbsSurface {
            order_u:  degree_u + 1,
            order_v:  degree_v + 1,
            degree_u: degree_u,
            degree_v: degree_v,
            knotvec_u: knot_vector_u.clone(),
            knotvec_v: knot_vector_v.clone(),
            ctrlpts_w: ctrlpts_w.clone(),
            ctrlpts: control_points.clone(),
            weights: weight.clone(),
            ctrlpts_size_u: size_u,
            ctrlpts_size_v: size_v,
            delta: 0.0,
            delta_u: 0.0,
            delta_v: 0.0,
            sample_size: 0,
            sample_size_u: 100,
            sample_size_v: 100,
            bbox: 0,
            name: "".to_string(),
            dimension: 2,
            vis: 0,
            rational: true,
            bspline: BSplineSurface::<U4>::new(ctrlpts_w, knot_vector_u, knot_vector_v, degree_u, degree_v, size_u, size_v),
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
                let coord = self.bspline.evaluate_single(u, v);
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

    /// Splits the surface at the input parametric coordinate on
    /// the u-direction.
    ///
    /// This method splits the surface into two pieces at the given
    /// parametric coordinate on the u-direction, generates two
    /// different surface objects and returns them. It does not modify
    /// the input surface.
    ///
    /// # Arguments
    ///
    /// - `u`: param for the u-direction
    pub fn split_surface_u(&self, u: Scalar) -> (NurbsSurface, NurbsSurface) {

        // Find multiplicity of the knot
        let span = helper::find_span(self.ctrlpts_size_u, self.degree_u, u, &self.knotvec_u);
        let ks = span - self.degree_u + 1;
        let s = helper::find_multiplicity(&self.knotvec_u, u);
        let r = self.degree_u - s;

        // Create backups of the original surface
        let mut temp = self.clone();

        // Split the original surface
        temp.bspline.insert_knot((Some(u), None), (r, 0));

        // Knot vectors
        let knot_span = helper::find_span(
            temp.bspline.size_u, temp.degree_u, u, &temp.bspline.knotvec_u) + 1;

        let mut surf1_kv = temp.bspline.knotvec_u[0..knot_span].to_vec();
        surf1_kv.push(u);
        let mut surf2_kv = temp.bspline.knotvec_u[knot_span..].to_vec();
        for _ in 0..=temp.degree_u {
            surf2_kv.insert(0, u);
        }

        // Control points
        let mut surf1_weights = Vec::new();
        let mut surf1_ctrlpts = Vec::new();
        let surf1_ctrlpts_end = (ks + r) * temp.ctrlpts_size_v;
        for point in &temp.bspline.ctrlpts[0..surf1_ctrlpts_end] {
            surf1_ctrlpts.push(Vector3f::new(
                point[0] / point[3], point[1] / point[3], point[2] / point[3]));
            surf1_weights.push(point[3]);
        }

        let mut surf2_ctrlpts = Vec::new();
        let mut surf2_weights = Vec::new();
        let surf2_ctrlpts_beg = (ks + r - 1) * temp.ctrlpts_size_v;
        for point in &temp.bspline.ctrlpts[surf2_ctrlpts_beg..] {
            surf2_ctrlpts.push(Vector3f::new(
                point[0] / point[3], point[1] / point[3], point[2] / point[3]));
            surf2_weights.push(point[3]);
        }

        // Create a new surface for the first half
        let surf1 = NurbsSurface::new(
            surf1_ctrlpts,
            surf1_kv,
            temp.knotvec_v.clone(),
            temp.degree_u,
            temp.degree_v,
            ks + r,
            temp.ctrlpts_size_v,
            surf1_weights,
        );

        // Create another surface fot the second half
        let surf2 = NurbsSurface::new(
            surf2_ctrlpts,
            surf2_kv,
            temp.knotvec_v.clone(),
            temp.degree_u,
            temp.degree_v,
            temp.bspline.size_u - ks - r + 1,
            temp.ctrlpts_size_v,
            surf2_weights,
        );

        // Return the new surfaces
        let ret_val = (surf1, surf2);
        return ret_val
    }

    /// Splits the surface at the input parametric coordinate on
    /// the u-direction.
    ///
    /// This method splits the surface into two pieces at the given
    /// parametric coordinate on the v-direction, generates two
    /// different surface objects and returns them. It does not modify
    /// the input surface.
    ///
    /// # Arguments
    ///
    /// - `v`: param for the v-direction
    pub fn split_surface_v(&self, v: Scalar) -> (NurbsSurface, NurbsSurface) {

        // Find multiplicity of the knot
        let span = helper::find_span(self.ctrlpts_size_v, self.degree_v, v, &self.knotvec_v);
        let ks = span - self.degree_v + 1;
        let s = helper::find_multiplicity(&self.knotvec_v, v);
        let r = self.degree_v - s;

        // Create backups of the original surface
        let mut temp = self.clone();

        // Split the original surface
        temp.bspline.insert_knot((None, Some(v)), (0, r));

        // Knot vectors
        let knot_span = helper::find_span(
            temp.bspline.size_v, temp.degree_v, v, &temp.bspline.knotvec_v) + 1;

        let mut surf1_kv = temp.bspline.knotvec_v[0..knot_span].to_vec();
        surf1_kv.push(v);
        let mut surf2_kv = temp.bspline.knotvec_v[knot_span..].to_vec();
        for _ in 0..=temp.degree_v {
            surf2_kv.insert(0, v);
        }

        // Control points
        let mut surf1_weights = Vec::new();
        let mut surf1_ctrlpts = Vec::new();
        for iu in 0..temp.ctrlpts_size_u {
            for iv in 0..ks+r {
                let index = iu * temp.bspline.size_v + iv;
                let point = &temp.bspline.ctrlpts[index];
                surf1_ctrlpts.push(Vector3f::new(
                    point[0] / point[3], point[1] / point[3], point[2] / point[3]));
                surf1_weights.push(point[3]);
            }
        }

        let mut surf2_ctrlpts = Vec::new();
        let mut surf2_weights = Vec::new();
        for iu in 0..temp.ctrlpts_size_u {
            for iv in ks+r-1..temp.bspline.size_v {
                let index = iu * temp.bspline.size_v + iv;
                let point = &temp.bspline.ctrlpts[index];
                surf2_ctrlpts.push(Vector3f::new(
                    point[0] / point[3], point[1] / point[3], point[2] / point[3]));
                surf2_weights.push(point[3]);
            }
        }

        // Create a new surface for the first half
        let surf1 = NurbsSurface::new(
            surf1_ctrlpts,
            temp.knotvec_u.clone(),
            surf1_kv,
            temp.degree_u,
            temp.degree_v,
            temp.ctrlpts_size_u,
            ks + r,
            surf1_weights,
        );

        // Create another surface fot the second half
        let surf2 = NurbsSurface::new(
            surf2_ctrlpts,
            temp.knotvec_u.clone(),
            surf2_kv,
            temp.degree_u,
            temp.degree_v,
            temp.ctrlpts_size_u,
            temp.bspline.size_v - ks - r + 1,
            surf2_weights,
        );

        // Return the new surfaces
        let ret_val = (surf1, surf2);
        return ret_val
    }
}