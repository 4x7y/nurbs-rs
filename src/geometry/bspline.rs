use na::*;
use na::allocator::Allocator;

pub struct BSplineCurve<D: Dim + DimName>
    where DefaultAllocator: Allocator<f32, D> {
    pub ctrlpts: Vec<VectorN<f32, D>>,
    pub size: usize,
    pub knotvec: Vec<f32>,
    pub degree: usize,
}

pub struct BSplineSurface<D: Dim + DimName>
    where DefaultAllocator: Allocator<f32, D> {
    pub ctrlpts: Vec<VectorN<f32, D>>,
    pub knot_vector_u: Vec<f32>,
    pub knot_vector_v: Vec<f32>,
    pub degree_u: usize,
    pub degree_v: usize,
    pub size_u: usize,
    pub size_v: usize,
}


impl <D: Dim + DimName> BSplineCurve<D>
    where DefaultAllocator: Allocator<f32, D> {

    /// Create a B-Spline curve given knots
    pub fn new(degree: usize, control_points: Vec<VectorN<f32, D>>, knot_vector: Vec<f32>) -> Self {
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
    pub fn find_span(n: usize, p: usize, u: f32, knot_vec: &Vec<f32>) -> usize {
        if (u as f32 - knot_vec[n+1]).abs() < 1e-6 {
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
    pub fn basis_functions(i: usize, u: f32, p: usize, knot_vec: &Vec<f32>) -> Vec<f32> {
        let mut basis: Vec<f32> = vec![0f32; p+1];
        let mut  left: Vec<f32> = vec![0f32; p+1];
        let mut right: Vec<f32> = vec![0f32; p+1];

        basis[0] = 1f32;
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
                                      i: usize, p: usize, u: f32,
                                      n: usize, knotvec: Vec<f32>) -> Vec<Vec<f32>> {
        let mut left  = vec![1f32; p+1];
        let mut right = vec![1f32; p+1];
        // `ndu` stores the basis functions and knot differences
        // ndu[0][0] = 1.0 by definition
        let mut ndu   = vec![vec![1f32; p+1]; p+1];
        // `a` store the two most recently computed rows a_{k, j} and a_{k-1, j} in
        // an alternating fashion
        let mut a     = vec![vec![0f32; 2]; p+1];
        // derivatives, ders[k][j] is the k-th derivative of the function N_{i-p+j, p}
        // where 0 <= k <= n, 0 <= j <= p.
        let mut ders  = vec![vec![0f32; p+1]; min(p, n)+1];

        for j in 1..=p {
            left[j]  = u - knotvec[i+1-j];
            right[j] = knotvec[i+1] - u;
            let mut saved = 0f32;
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

            a[0][0] = 1f32;
            // loop to compute the k-th derivative
            for k in 1..=n {
                let mut d = 0f32;
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
                ders[k][j] *= r as f32;
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
    pub fn curve_point(&self, u: f32) -> VectorN<f32, D> {
        let span = BSplineCurve::find_span(self.size, self.degree, u, &self.knotvec);
        let basis = BSplineCurve::basis_functions(span, u, self.degree, &self.knotvec);
        let mut point: VectorN<f32, D> = VectorN::zeros();
        for i in 0..=self.degree {
            point = point + basis[i] * &self.ctrlpts[span-self.degree+i];
        }
        return point;
    }
}

impl <D: Dim + DimName> BSplineSurface<D>
    where DefaultAllocator: Allocator<f32, D> {

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
    pub fn new(control_points: Vec<VectorN<f32, D>>,
               knot_vector_u: Vec<f32>,
               knot_vector_v: Vec<f32>,
               degree_u: usize,
               degree_v: usize,
               size_u: usize,
               size_v: usize,) -> Self {

        assert_eq!(size_u * size_v, control_points.len());

        BSplineSurface {
            degree_u,
            degree_v,
            ctrlpts: control_points,
            knot_vector_u,
            knot_vector_v,
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
    pub fn evaluate_single(&self, u: f32, v: f32) -> VectorN<f32, D> {
        let span_u = BSplineCurve::find_span(self.size_u, self.degree_u, u, &self.knot_vector_u);
        let span_v = BSplineCurve::find_span(self.size_v, self.degree_v, v, &self.knot_vector_v);
        let basis_u = BSplineCurve::basis_functions(span_u, u, self.degree_u, &self.knot_vector_u);
        let basis_v = BSplineCurve::basis_functions(span_v, v, self.degree_v, &self.knot_vector_v);

        let index_u = span_u - self.degree_u;
        let mut point: VectorN<f32, D> = VectorN::zeros();
        for j in 0..=self.degree_v {
            let mut temp: VectorN<f32, D> = VectorN::zeros();
            let index_v = span_v - self.degree_v + j;
            for k in 0..=self.degree_u {
                let index_ctrl_point = (index_u + k) * self.size_v + index_v;
                temp = temp + basis_u[k] * &self.ctrlpts[index_ctrl_point];
            }
            point = point + basis_v[j] * temp;
        }

        return point;
    }
}
