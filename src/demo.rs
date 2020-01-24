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
    let bezier_curve = bezier::BezierCurve::new(&control_points);
    let curve = bezier_curve.get_curve(100);

    let mut window = Window::new("[ME 625] Bezier Demo");
    window.set_light(Light::StickToCamera);

    while window.render() {
        let red   = Point3::new(1.0, 0.0, 0.0);
        let white = Point3::new(1.0, 1.0, 1.0);

        for i in 1..=100 {
            let s = Point3::from(curve[i - 1]);
            let t = Point3::from(curve[i]);
            window.draw_line(&s, &t, &red);
        }

        window.draw_line(&Point3::from(control_points[0]), &Point3::from(control_points[1]), &white);
        window.draw_line(&Point3::from(control_points[1]), &Point3::from(control_points[2]), &white);
        window.draw_line(&Point3::from(control_points[2]), &Point3::from(control_points[3]), &white);
        window.draw_line(&Point3::from(control_points[3]), &Point3::from(control_points[0]), &white);
        window.draw_line(&Point3::from(control_points[0]), &Point3::from(control_points[2]), &white);
        window.draw_line(&Point3::from(control_points[0]), &Point3::from(control_points[3]), &white);
        window.draw_line(&Point3::from(control_points[1]), &Point3::from(control_points[3]), &white);

        sleep(30);
    }
}

fn main() {
    bezier_tests();
}
