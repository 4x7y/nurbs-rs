extern crate nalgebra as na;

use crobot::geometry::bezier;
use kiss3d::light::Light;
use kiss3d::window::Window;
use na::{Point3, Vector3};
use std::{thread, time};

fn sleep(millis: u64) {
    let duration = time::Duration::from_millis(millis);
    thread::sleep(duration);
}

fn bezier_tests() {
    let control_points = vec![
        Vector3::new(-0.1, -0.1, 0.0),
        Vector3::new(0.0, 0.1, 0.0),
        Vector3::new(0.1, -0.1, 0.1),
        Vector3::new(-0.1, 0.0, 0.3),
    ];
    let weights = vec![1., 20., 1., 4.];


    let bezier_curve = bezier::BezierCurve::new(control_points.clone());
    let bc = bezier_curve.get_curve(100);
    let rational_bezier_curve = bezier::RationalBezierCurve::new(control_points, weights);
    let rbc = rational_bezier_curve.get_curve(100);

    let mut window = Window::new("[ME 625] Bezier Demo");
    window.set_light(Light::StickToCamera);

    while window.render() {
        let red   = Point3::new(1.0, 0.0, 0.0);

        for i in 1..=100 {
            let s = Point3::from(bc[i - 1]);
            let t = Point3::from(bc[i]);
            window.draw_line(&s, &t, &red);
        }

        for i in 1..=100 {
            let s = Point3::from(rbc[i - 1]);
            let t = Point3::from(rbc[i]);
            window.draw_line(&s, &t, &red);
        }
        sleep(30);
    }
}

fn main() {
    bezier_tests();
}
