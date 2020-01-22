extern crate alga;
use na::{DVector, Vector3};

/// Compute point on power basis curve.
pub fn horner(a: &Vec<f32>, n: usize, u: f32) -> f32 {
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
pub fn all_bernstein(n: usize, u: f32) -> DVector<f32> {
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
pub fn point_on_bezier_curve(control_points: &Vec<Vector3<f32>>, n: usize, u: f32) -> Vector3<f32> {
    let bernstein = all_bernstein(n, u);
    let mut point = Vector3::zeros();
    for k in 0..=n {
        point = point + bernstein[k] * &control_points[k];
    }
    return point;
}

/// Compute n-th order bezier curve.
pub fn bezier(control_points: &Vec<Vector3<f32>>, n: usize, m: usize) -> Vec<Vector3<f32>> {
    let mut curve: Vec<Vector3<f32>> = Vec::new();
    for i in 0..=m {
        let u: f32 = (i as f32) / (m as f32);
        curve.push(point_on_bezier_curve(control_points, n, u));
    }

    return curve;
}


// /// Given a set of control points, generates a (non-rational) Bezier surface.
// #[cfg(feature = "dim3")]
// pub fn bezier_surface<N>(
//     control_points: &[Point<N>],
//     nupoints: usize,
//     nvpoints: usize,
//     usubdivs: usize,
//     vsubdivs: usize,
// ) -> TriMesh<N>
// where
//     N: RealField,
// {
//     assert!(nupoints * nvpoints == control_points.len());

//     let mut surface = super::unit_quad(usubdivs, vsubdivs);

//     {
//         let uvs = &surface.uvs.as_ref().unwrap()[..];
//         let coords = &mut surface.coords[..];

//         let mut ucache = Vec::new();
//         let mut vcache = Vec::new();

//         for j in 0..vsubdivs + 1 {
//             for i in 0..usubdivs + 1 {
//                 let id = i + j * (usubdivs + 1);
//                 coords[id] = bezier_surface_at(
//                     control_points,
//                     nupoints,
//                     nvpoints,
//                     uvs[id].x,
//                     uvs[id].y,
//                     &mut ucache,
//                     &mut vcache,
//                 )
//             }
//         }

//         // XXX: compute the normals manually.
//         surface.normals = None;
//     }

//     surface
// }