extern crate nalgebra as na;
extern crate alga;
use na::{DVector};


pub fn bernstein(i: usize, n: usize, u: f64) -> f64 {
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

pub fn all_bernstein(n: usize, u: f64) -> DVector<f64> {
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

pub fn point_on_bezier_curve(control_points: Vec<DVector<f64>>, n: usize, u: f64) -> DVector<f64> {
    let bernstein = all_bernstein(n, u);
    let dim = control_points[0].nrows();
    let mut point = DVector::zeros(dim);
    for k in 0..=n {
        point = point + bernstein[k] * &control_points[k];
    }
    return point;
}
