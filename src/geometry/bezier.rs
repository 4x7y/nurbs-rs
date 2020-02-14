extern crate alga;
use na::*;
use na::allocator::Allocator;

pub struct BezierCurve<D: Dim + DimName>
    where DefaultAllocator: Allocator<f32, D> {
    pub ctrlpts: Vec<VectorN<f32, D>>,
    pub degree: usize,
}

pub struct RationalBezierCurve {
    pub bezier_curve: BezierCurve<U4>,
}

pub struct LineSegment {
    pub start: Vector3<f32>,
    pub end: Vector3<f32>
}

/// 3D Cartesian coordinates to Homogeneous coordinates
pub fn cart2homo(vec: &Vector3<f32>) -> Vector4<f32> {
    return Vector4::new(vec[0], vec[1], vec[2], 1.0);
}

/// 3D Cartesian space vector to Homogeneous coordinates
pub fn xvec2homo(vec: &Vector3<f32>) -> Vector4<f32> {
    return Vector4::new(vec[0], vec[1], vec[2], 0.0);
}

/// Homogeneous coordinates to 3D Cartesian coordinates
pub fn homo2cart(vec: &Vector4<f32>) -> Option<Vector3<f32>> {
    if vec[3] == 0.0 {
        return None;
    } else {
        return Some(Vector3::new(vec[0], vec[1], vec[2]) / vec[3]);
    }
}

/// Homogeneous coordinates to 3D Cartesian space vector
pub fn homo2xvec(vec: &Vector4<f32>) -> Option<Vector3<f32>> {
    if vec[3] == 0.0 {
        return Some(Vector3::new(vec[0], vec[1], vec[2]) / vec[3]);
    } else {
        return None;
    }
}

impl RationalBezierCurve {

    /// Create a Rational Bezier curve given control points and weights
    pub fn new(control_points: Vec<Vector3<f32>>, weights: Vec<f32>) -> Self {
        let mut control_points_homo = Vec::new();
        for i in 0..control_points.len() {
            control_points_homo.push(cart2homo(&control_points[i]) * weights[i]);
        }
        RationalBezierCurve {
            bezier_curve: BezierCurve::new(control_points_homo),
        }
    }

    /// Compute n-th order rational bezier curve.
    pub fn get_curve(&self, m: usize) -> Vec<Vector3<f32>> {
        let curve_homo = self.bezier_curve.get_curve(m);
        let mut curve = Vec::new();
        for point in curve_homo {
            curve.push(homo2cart(&point).unwrap());
        }
        return curve;
    }
}


impl<D: Dim + DimName> BezierCurve<D>
    where DefaultAllocator: Allocator<f32, D> {

    /// Create a Bezier curve given control points
    pub fn new(control_points: Vec<VectorN<f32, D>>) -> Self {
        BezierCurve {
            degree: control_points.len() - 1,
            ctrlpts: control_points,
        }
    }

    /// Compute point on power basis curve.
    pub fn evaluate_polynomial(a: &Vec<f32>, n: usize, u: f32) -> f32 {
        let mut c = a[n];
        for i in (0..n).rev() {
            c = c * u + a[i];
        }
        return c;
    }


    /// Compute value of the i-th Bernstein polynomial of order n at fixed u.
    pub fn bernstein(i: usize, n: usize, u: f32) -> f32 {
        let mut temp = DVector::zeros(n + 1);
        temp[n - i] = 1.;
        let u1 = 1. - u;
        for k in 1..=n {
            for j in (k..=n).rev() {
                temp[j] = u1 * temp[j] + u * temp[j - 1];
            }
        }

        return temp[n];
    }


    /// Compute values of all the Bernstein polynomials of order n at fixed u.
    pub fn all_bernstein(&self, u: f32) -> DVector<f32> {
        let n = self.degree;
        let mut bernstein = DVector::zeros(n + 1);
        bernstein[0] = 1.;
        let u1 = 1. - u;

        for j in 1..=n {
            let mut saved = 0.;
            for k in 0..j {
                let temp = bernstein[k];
                bernstein[k] = saved + u1 * temp;
                saved = u * temp;
            }
            bernstein[j] = saved;
        }

        return bernstein;
    }


    /// Compute the point on the n-th order Bezier curve at fixed u.
    pub fn point_on_bezier_curve(&self, u: f32) -> VectorN<f32, D> {
        let n = self.degree;
        let bernstein = self.all_bernstein(u);
        let mut point = VectorN::zeros();
        for k in 0..=n {
            point = point + bernstein[k] * &self.ctrlpts[k];
        }
        return point;
    }


    /// De Casteljau Algorithm
    pub fn de_casteljau(&self, u: f32) -> VectorN<f32, D> {
        let mut points = self.ctrlpts.clone();
        let n = self.degree;
        for i in 1..=n {
            for k in 0..=n-i {
                points[k] = (1. - u) * &points[k] + u * &points[k+1];
            }
        }

        return points[0].clone();
    }

    /// Compute n-th order bezier curve.
    pub fn get_curve(&self, m: usize) -> Vec<VectorN<f32, D>> {
        let mut curve: Vec<VectorN<f32, D>> = Vec::new();
        for i in 0..=m {
            let u: f32 = (i as f32) / (m as f32);
            curve.push(self.de_casteljau(u));
        }

        return curve;
    }

    pub fn control_polygon(&self) -> Vec<LineSegment> {
        unimplemented!()
    }
}
