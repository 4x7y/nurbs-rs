extern crate alga;
use na::{DVector, Vector3};

pub struct BezierCurve {
    pub control_points: Vec<Vector3<f32>>,
    pub degree: usize,
}

pub struct LineSegment {
    pub start: Vector3<f32>,
    pub end: Vector3<f32>
}

impl BezierCurve {

    /// Create a Bezier curve given control points
    pub fn new(control_points: &Vec<Vector3<f32>>) -> BezierCurve {
        BezierCurve {
            control_points: control_points.clone(),
            degree: control_points.len() - 1,
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
    pub fn point_on_bezier_curve(&self, u: f32) -> Vector3<f32> {
        let n = self.degree;
        let bernstein = self.all_bernstein(u);
        let mut point = Vector3::zeros();
        for k in 0..=n {
            point = point + bernstein[k] * &self.control_points[k];
        }
        return point;
    }


    /// De Casteljau Algorithm
    pub fn de_casteljau(&self, u: f32) -> Vector3<f32> {
        let mut points = self.control_points.clone();
        let n = self.degree;
        for i in 1..=n {
            for k in 0..=n-i {
                points[k] = (1. - u) * points[k] + u * points[k+1];
            }
        }

        return points[0];
    }

    /// Compute n-th order bezier curve.
    pub fn get_curve(&self, m: usize) -> Vec<Vector3<f32>> {
        let mut curve: Vec<Vector3<f32>> = Vec::new();
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
