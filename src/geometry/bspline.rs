use na::*;
use na::allocator::Allocator;
use crate::math::Scalar;
use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::resource::Mesh;

#[derive(Debug, Clone)]
pub struct BSplineCurve<D: Dim + DimName>
    where DefaultAllocator: Allocator<Scalar, D> {
    pub ctrlpts: Vec<VectorN<Scalar, D>>,
    pub size: usize,
    pub knotvec: Vec<Scalar>,
    pub degree: usize,
}

#[derive(Debug, Clone)]
pub struct BSplineSurface<D: Dim + DimName>
    where DefaultAllocator: Allocator<Scalar, D> {
    pub ctrlpts: Vec<VectorN<Scalar, D>>,
    pub knotvec_u: Vec<Scalar>,
    pub knotvec_v: Vec<Scalar>,
    pub degree_u: usize,
    pub degree_v: usize,
    pub size_u: usize,
    pub size_v: usize,
}


impl <D: Dim + DimName> BSplineCurve<D>
    where DefaultAllocator: Allocator<Scalar, D> {

    /// Create a B-Spline curve given knots
    pub fn new(degree: usize, control_points: Vec<VectorN<Scalar, D>>, knot_vector: Vec<Scalar>) -> Self {
        BSplineCurve {
            size: control_points.len(),
            degree,
            ctrlpts: control_points,
            knotvec: knot_vector,
        }
    }

    /// Determine the knot span index where u lies
    ///
    /// # Arguments
    ///
    /// * `n` - number of control points
    /// * `p` - basis function degree
    /// * `u` - parameter
    /// * `knot_vec` - knot vector
    pub fn find_span(n: usize, p: usize, u: Scalar, knot_vec: &Vec<Scalar>) -> usize {
        if (u - knot_vec[n+1]).abs() < 1e-6 {
            return n;
        }

        // Bisection search
        let mut low: usize  = p;
        let mut high: usize = n+1;
        let mut mid: usize  =  (low + high) / 2;
        while u < knot_vec[mid] || u >= knot_vec[mid+1] {
            if u < knot_vec[mid] {
                high = mid;
            } else {
                low = mid;
            }
            mid = (low + high) / 2;
        }

        return mid;
    }


    /// Compute all the non-vanishing basis functions (no division by zero)
    ///
    /// Implementation of Algorithm A2.2 from The NURBS Book by Piegl & Tiller.
    /// Uses recurrence to compute the basis functions, also known as Cox - de
    /// Boor recursion formula.
    ///
    /// # Arguments
    ///
    /// * `i` - knot span
    /// * `u` - parameter
    /// * `p` - basis function degree
    /// * `knot_vec` - knot vector
    pub fn basis_functions(i: usize, u: Scalar, p: usize, knot_vec: &Vec<Scalar>) -> Vec<Scalar> {
        let mut basis: Vec<Scalar> = vec![0.; p+1];
        let mut  left: Vec<Scalar> = vec![0.; p+1];
        let mut right: Vec<Scalar> = vec![0.; p+1];

        basis[0] = 1.;
        for j in 1..=p {
            left[j]  = u - knot_vec[i+1-j];
            right[j] = knot_vec[i+j] - u;
            let mut saved = 0.;
            for r in 0..j {
                let temp = basis[r] / (right[r+1] + left[j-r]);
                basis[r] = saved + right[r+1] * temp;
                saved = left[j-r] * temp;
            }
            basis[j] = saved;
        }

        return basis;
    }

    /// Computes derivatives of the basis functions for a single parameter.
    /// Return a 2D array `D`. `D[k][j]` is the k-th derivative of the
    /// function $N_{i-p+j, p}$, where $0 \leq k \leq n, 0 \leq j \leq p$.
    ///
    /// # Arguments
    ///
    /// * `i` - knot span
    /// * `p` - degree
    /// * `u` - parameter
    /// * `n` - order of the derivatives
    /// * `knotvec` - knot vector
    pub fn basis_function_derivatives(&self,
                                      i: usize, p: usize, u: Scalar,
                                      n: usize, knotvec: Vec<Scalar>) -> Vec<Vec<Scalar>> {
        let mut left  = vec![1.; p+1];
        let mut right = vec![1.; p+1];
        // `ndu` stores the basis functions and knot differences
        // ndu[0][0] = 1.0 by definition
        let mut ndu   = vec![vec![1.; p+1]; p+1];
        // `a` store the two most recently computed rows a_{k, j} and a_{k-1, j} in
        // an alternating fashion
        let mut a     = vec![vec![0.; 2]; p+1];
        // derivatives, ders[k][j] is the k-th derivative of the function N_{i-p+j, p}
        // where 0 <= k <= n, 0 <= j <= p.
        let mut ders  = vec![vec![0.; p+1]; min(p, n)+1];

        for j in 1..=p {
            left[j]  = u - knotvec[i+1-j];
            right[j] = knotvec[i+1] - u;
            let mut saved = 0.;
            for r in 0..j {
                // lower triangle
                ndu[j][r] = right[r+1] + left[j-r];
                let temp = ndu[r][j-1] / ndu[j][r];

                // upper triangle
                ndu[r][j] = saved + right[r+1] * temp;
                saved = left[j-r] * temp;
            }
            ndu[j][j] = saved;
        }

        // load the basis functions
        for j in 0..=p {
            ders[0][j] = ndu[j][p];
        }

        // This section computes the derivatives (Eq. 2.9)
        // loop over function index
        for r in 0..=p {
            let mut s1 = 0usize;
            let mut s2 = 0usize;

            a[0][0] = 1.;
            // loop to compute the k-th derivative
            for k in 1..=n {
                let mut d = 0.;
                let r_k = r - k;
                let p_k = p - k;

                if r >= k {
                    a[s2][0] = a[s1][0] / ndu[p_k + 1][r_k];
                    d = a[s2][0] * ndu[r_k][p_k];
                }

                let j1 = if r   >= k-1 {   1 } else { k-r };
                let j2 = if r-1 <= p_k { k-1 } else { p-r };
                for j in j1..=j2 {
                    a[s2][j] = (a[s1][j] - a[s1][j-1]) / ndu[p_k+1][r_k+j];
                    d += a[s2][j] * ndu[r_k+j][p_k];
                }

                if r <= p_k {
                    a[s2][k] = -a[s1][k-1] / ndu[p_k+1][r];
                    d += a[s2][k] * ndu[r][p_k];
                }
                ders[k][r] = d;

                // switch rows
                let j = s1; s1 = s2; s2 = j;
            }
        }

        // Multiply through by the correct factors (Eq. 2.9)
        let mut r = p;
        for k in 1..=n {
            for j in 0..=p {
                ders[k][j] *= r as Scalar;
            }
            r *= p - k;
        }

        return ders;
    }


    /// Compute curve point
    ///
    /// # Arguments
    ///
    /// * `u` - parameter
    pub fn curve_point(&self, u: Scalar) -> VectorN<Scalar, D> {
        let span = BSplineCurve::find_span(self.size, self.degree, u, &self.knotvec);
        let basis = BSplineCurve::basis_functions(span, u, self.degree, &self.knotvec);
        let mut point: VectorN<Scalar, D> = VectorN::zeros();
        for i in 0..=self.degree {
            point = point + basis[i] * &self.ctrlpts[span-self.degree+i];
        }
        return point;
    }

    pub fn find_multiplicity(knot_vec: &Vec<Scalar>, u: Scalar) -> usize {
        let tol: Scalar = 1e-6;
        let mut multiplicity = 0;
        for knot in knot_vec {
            if (knot - u).abs() < tol {
                multiplicity += 1;
            }
        }

        return multiplicity;
    }


    /// Inserts knots n-times to a spline
    ///
    /// # Arguments
    ///
    /// - `u`: knot
    /// - `r`: number of knot insertions
    pub fn insert_knot(&mut self, u: Scalar, r: usize) {
        let s = Self::find_multiplicity(&self.knotvec, u);
        // find knot span
        let span = Self::find_span(self.size, self.degree, u, &self.knotvec);
        // load new knot vector
        let knotvec_new = Self::knotvec_insert_knot(&self.knotvec, u, span, r);
        // compute new control points
        let ctrlpts_new = Self::ctrlpts_insert_knot(&self.knotvec, &self.ctrlpts, self.degree, u, r, s, span);
        // set new control points and knot vector
        self.ctrlpts = ctrlpts_new;
        self.knotvec = knotvec_new;
    }

    /// Computes the knot vector of the rational/non-rational spline
    /// after knot insertion.
    ///
    /// # Arguments
    ///
    /// - `knot_vec`: knot vector
    /// - `u`: knot
    /// - `span`: knot span
    /// - `r`: number of knot insertions
    ///
    /// # Return
    ///
    /// `knot_vec_new`: updated knot vector
    pub fn knotvec_insert_knot(knot_vec: &Vec<Scalar>,
                               u: Scalar, span: usize, r: usize) -> Vec<Scalar> {
        let np = knot_vec.len();
        let nq = np + r;
        let mut knot_vec_new = vec![0.; nq];

        // compute new knot vec
        for i in 0..=span {
            knot_vec_new[i] = knot_vec[i]
        }
        for i in 1..=r {
            knot_vec_new[span + i] = u;
        }
        for i in span+1..np {
            knot_vec_new[i+r] = knot_vec[i];
        }

        return knot_vec_new;
    }


    /// Computes the control points of the rational/non-rational spline
    /// after knot insertion.
    ///
    /// # Arguments
    ///
    /// - `knot_vec`: knot vector
    /// - `ctrlpts`: control points
    /// - `u`: knot to be inserted
    /// - `r`: number of knot insertions
    /// - `s`: multiplicity
    /// - `k`: knot span
    ///
    /// # Return
    ///
    /// - `ctrlpts_new`: updated control points
    pub fn ctrlpts_insert_knot(knot_vec: &Vec<Scalar>,
                               ctrlpts: &Vec<VectorN<Scalar, D>>,
                               degree: usize,
                               u: Scalar, num: usize, s: usize, k: usize) -> Vec<VectorN<Scalar, D>> {
        let np = ctrlpts.len();
        let nq = np + num;

        let mut ctrlpts_new = vec![VectorN::<Scalar, D>::zeros(); nq];
        let mut temp = vec![VectorN::<Scalar, D>::zeros(); degree+1];

        for i in 0..=k-degree {
            ctrlpts_new[i] = ctrlpts[i].clone_owned();
        }
        for i in k-s..np {
            ctrlpts_new[i + num] = ctrlpts[i].clone_owned();
        }
        for i in 0..=degree-s {
            temp[i] = ctrlpts[k-degree+i].clone_owned();
        }
        for j in 1..=num {
            let l = k - degree + j;
            for i in 0..=degree-j-s {
                let alpha = (u - knot_vec[l+i]) / (knot_vec[i+k+1] - knot_vec[l+i]);
                temp[i] = alpha * temp[i+1].clone_owned() + (1. - alpha) * &temp[i];
            }
            ctrlpts_new[l] = temp[0].clone_owned();
            ctrlpts_new[k+num-j-s] = temp[degree-j-s].clone_owned();
        }

        let l = k - degree + num;
        for i in l+1..k-s {
            ctrlpts_new[i] = temp[i-l].clone_owned();
        }

        return ctrlpts_new;
    }
}

impl <D: Dim + DimName> BSplineSurface<D>
    where DefaultAllocator: Allocator<Scalar, D> {

    /// Create a B-Spline surface
    ///
    /// # Arguments
    ///
    /// * `control_points` - control points
    /// * `knot_vector_u` - knot vector for parameter u
    /// * `knot_vector_v` - knot vector for parameter v
    /// * `degree_u` - degree for parameter u
    /// * `degree_v` - degree for parameter v
    /// * `size_u` - number of control points along u
    /// * `size_v` - number of control points along v
    pub fn new(control_points: Vec<VectorN<Scalar, D>>,
               knot_vector_u: Vec<Scalar>,
               knot_vector_v: Vec<Scalar>,
               degree_u: usize,
               degree_v: usize,
               size_u: usize,
               size_v: usize,) -> Self {

        assert_eq!(size_u * size_v, control_points.len());

        BSplineSurface {
            degree_u,
            degree_v,
            ctrlpts: control_points,
            knotvec_u: knot_vector_u,
            knotvec_v: knot_vector_v,
            size_u,
            size_v,
        }
    }

    /// Evaluates the surface at the input (u, v) parameter pair.
    /// Algorithm A3.5: SurfacePoint
    ///
    /// # Arguments
    ///
    /// * `u` - the first parameter
    /// * `v` - the second parameter
    pub fn evaluate_single(&self, u: Scalar, v: Scalar) -> VectorN<Scalar, D> {
        let span_u = BSplineCurve::find_span(self.size_u, self.degree_u, u, &self.knotvec_u);
        let span_v = BSplineCurve::find_span(self.size_v, self.degree_v, v, &self.knotvec_v);
        let basis_u = BSplineCurve::basis_functions(span_u, u, self.degree_u, &self.knotvec_u);
        let basis_v = BSplineCurve::basis_functions(span_v, v, self.degree_v, &self.knotvec_v);

        let index_u = span_u - self.degree_u;
        let mut point: VectorN<Scalar, D> = VectorN::zeros();
        for j in 0..=self.degree_v {
            let mut temp: VectorN<Scalar, D> = VectorN::zeros();
            let index_v = span_v - self.degree_v + j;
            for k in 0..=self.degree_u {
                let index_ctrl_point = (index_u + k) * self.size_v + index_v;
                temp = temp + basis_u[k] * &self.ctrlpts[index_ctrl_point];
            }
            point = point + basis_v[j] * temp;
        }

        return point;
    }

    pub fn insert_knot(&mut self, uv: (Option<Scalar>, Option<Scalar>), num: (usize, usize)) {
        if let Some(u) = uv.0 {
            let r = num.0;
            // find knot multiplicity
            let s = BSplineCurve::find_multiplicity(&self.knotvec_u, u);
            // find knot span
            let span = BSplineCurve::find_span(self.size_u, self.degree_u, u, &self.knotvec_u);
            // compute new knot vector
            let knotvec_u_new = BSplineCurve::knotvec_insert_knot(&self.knotvec_u, u, span, r);

            let mut ctrlpts_tmp = Vec::new();
            let ctrlpts = &self.ctrlpts;
            for j in 0..self.size_v {
                let mut ccu = Vec::new();
                for i in 0..self.size_u {
                    ccu.push(ctrlpts[j + self.size_v * i].clone_owned());
                }
                let mut tmp = BSplineCurve::ctrlpts_insert_knot(
                    &self.knotvec_u, &ccu, self.degree_u, u, r, s, span);
                ctrlpts_tmp.append(&mut tmp);
            }

            let ctrlpts_new = Self::flip_ctrlpts_u(ctrlpts_tmp, self.size_u + r, self.size_v);

            self.knotvec_u = knotvec_u_new;
            self.ctrlpts = ctrlpts_new;
            self.size_u = self.size_u + r;
        }

        if let Some(v) = uv.1 {
            let r = num.1;
            // find knot multiplicity
            let s = BSplineCurve::find_multiplicity(&self.knotvec_v, v);
            // find knot span
            let span = BSplineCurve::find_span(self.size_v, self.degree_v, v, &self.knotvec_v);
            // compute new knot vector
            let knotvec_v_new = BSplineCurve::knotvec_insert_knot(&self.knotvec_v, v, span, r);

            let mut ctrlpts_new = Vec::new();
            let ctrlpts = &self.ctrlpts;
            for u in 0..self.size_u {
                let mut ccv = Vec::new();
                for v in 0..self.size_v {
                    ccv.push(ctrlpts[v + self.size_v * u].clone_owned());
                }
                let mut tmp = BSplineCurve::ctrlpts_insert_knot(
                    &self.knotvec_v, &ccv, self.degree_v, v, r, s, span);
                ctrlpts_new.append(&mut tmp);
            }

            self.knotvec_v = knotvec_v_new;
            self.ctrlpts = ctrlpts_new;
            self.size_v = self.size_v + r;
        }
    }

    fn flip_ctrlpts_u(ctrlpts: Vec<VectorN<Scalar, D>>,
                      size_u: usize, size_v: usize) -> Vec<VectorN<Scalar, D>> {
        let mut ctrlpts_new = Vec::new();
        for i in 0..size_u {
            for j  in 0..size_v {
                ctrlpts_new.push(ctrlpts[i + j * size_u].clone_owned());
            }
        }
        return ctrlpts_new;
    }

    pub fn get_mesh(&self) -> Rc<RefCell<Mesh>> {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        for i in 0u16..100 {
            for j in 0u16..100 {
                let u = i as Scalar / 100.0;
                let v = j as Scalar / 100.0;
                let coord = self.evaluate_single(u, v);
                vertices.push(Point3::new(
                    coord[0] as f32, coord[1] as f32, coord[2] as f32));

                if i > 0 && j > 0 {
                    let idx_1 = i * 100 + j;
                    let idx_2 = (i - 1) * 100 + j;
                    let idx_3 = i * 100 + (j - 1);
                    indices.push(Point3::new(idx_1, idx_2, idx_3))
                }

                if i < 99 && j < 99 {
                    let idx_1 = i * 100 + j;
                    let idx_2 = (i + 1) * 100 + j;
                    let idx_3 = i * 100 + (j + 1);
                    indices.push(Point3::new(idx_1, idx_2, idx_3))
                }
            }
        }

        let mesh = Rc::new(RefCell::new(Mesh::new(
            vertices, indices, None, None, false,
        )));

        return mesh;
    }

    pub fn get_control_points(&self) -> Vec<VectorN<Scalar, D>> {
        return self.ctrlpts.clone();
    }

}
