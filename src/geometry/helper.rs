use na::*;
use na::allocator::Allocator;
use crate::math::Scalar;
use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::resource::Mesh;


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
pub fn ctrlpts_insert_knot<D: Dim + DimName>(
    knot_vec: &Vec<Scalar>,
    ctrlpts: &Vec<VectorN<Scalar, D>>,
    degree: usize,
    u: Scalar,
    num: usize,
    s: usize,
    k: usize) -> Vec<VectorN<Scalar, D>>
where DefaultAllocator: Allocator<Scalar, D> {
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

pub fn normalize_knotvec(knotvec: &Vec<Scalar>) -> Vec<Scalar> {
    let min = knotvec[0];
    let max = knotvec[knotvec.len() - 1];
    let range = max - min;
    let mut knotvec_norm = Vec::new();
    for i in 0..knotvec.len() {
        knotvec_norm.push((knotvec[i] - min) / range);
    }

    return knotvec_norm;
}