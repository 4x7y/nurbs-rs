extern crate nalgebra as na;

mod geometry;
mod robotics;
mod test;

use geometry::bezier;
use kiss3d::light::Light;
use kiss3d::window::Window;
use na::{Point3, Vector3};
use robotics::serial_link::SerialLink;

use std::{thread, time};

fn sleep(millis: u64) {
    let duration = time::Duration::from_millis(millis);
    thread::sleep(duration);
}

fn bezier_tests() {
    let coeffs = vec![1., 2., 3.];
    let p = bezier::horner(&coeffs, 2, 0.5);
    println!("{}", p);

    let bernstein = bezier::bernstein(0, 5, 0.3);
    println!("{}", bernstein);

    let all_bernstein = bezier::all_bernstein(5, 0.3);
    println!("{}", all_bernstein);

    let control_points = vec![
        Vector3::new(-0.1, -0.1, 0.0),
        Vector3::new(0.0, 0.1, 0.0),
        Vector3::new(0.1, -0.1, 0.1),
        Vector3::new(-0.1, 0.0, 0.3),
    ];
    let point = bezier::point_on_bezier_curve(&control_points, 3, 0.1);
    println!("{}", point);

    let curve = bezier::bezier(&control_points, 3, 100);

    let mut window = Window::new("Kiss3d: lines");
    window.set_light(Light::StickToCamera);

    while window.render() {
        for i in 1..100 {
            let s = Point3::from(curve[i - 1]);
            let t = Point3::from(curve[i]);
            window.draw_line(&s, &t, &Point3::new(1.0, 0.0, 0.0));
        }
        let a = Point3::new(-0.1, -0.1, 0.0);
        let b = Point3::new(0.0, 0.1, 0.0);
        let c = Point3::new(0.1, -0.1, 0.0);

        window.draw_line(&a, &b, &Point3::new(1.0, 0.0, 0.0));
        window.draw_line(&b, &c, &Point3::new(0.0, 1.0, 0.0));
        window.draw_line(&c, &a, &Point3::new(0.0, 0.0, 1.0));

        sleep(30);
    }
}

fn main() {
    bezier_tests();
}
