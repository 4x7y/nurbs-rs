extern crate nalgebra as na;

mod geometry;
mod test;

use na::DVector;
use geometry::bezier;

fn bezier_tests() {
    let bernstein = bezier::bernstein(0, 5, 0.3);
    println!("{}", bernstein);

    let all_bernstein = bezier::all_bernstein(5, 0.3);
    println!("{}", all_bernstein);

    let control_points = vec![
        DVector::from_row_slice(&[ 1.5,  5.8,  1.4]),
        DVector::from_row_slice(&[-4.7,  7.8, -2.1]),
        DVector::from_row_slice(&[ 0.2,  0.8,  6.6]),
        DVector::from_row_slice(&[-5.9, 17.2, -4.1]),
    ];
    let point = bezier::point_on_bezier_curve(control_points, 3, 0.1);
    println!("{}", point);
}

fn main() {
    bezier_tests();
}
