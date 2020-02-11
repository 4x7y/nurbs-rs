extern crate nalgebra as na;

use crobot::geometry::bezier;
use kiss3d::light::Light;
use kiss3d::window::Window;
use kiss3d::resource::Mesh;
use na::{Point3, Vector3, UnitQuaternion};
use std::{thread, time};
use std::cell::RefCell;
use std::rc::Rc;

fn sleep(millis: u64) {
    let duration = time::Duration::from_millis(millis);
    thread::sleep(duration);
}

fn bezier_tests() {
//    let control_points = vec![
//        Vector3::new(-0.1, -0.1, 0.0),
//        Vector3::new(0.0, 0.1, 0.0),
//        Vector3::new(0.1, -0.1, 0.1),
//        Vector3::new(-0.1, 0.0, 0.3),
//    ];
//    let weights = vec![1., 20., 1., 4.];
//
//
//    let bezier_curve = bezier::BezierCurve::new(control_points.clone());
//    let bc = bezier_curve.get_curve(100);
//    let rational_bezier_curve = bezier::RationalBezierCurve::new(control_points, weights);
//    let rbc = rational_bezier_curve.get_curve(100);

    let control_points = vec![
        Vector3::new(0.0, 0.0,  0.0),
        Vector3::new(0.0, 4.0,  0.0),
        Vector3::new(0.0, 8.0, -3.0),
        Vector3::new(2.0, 0.0,  6.0),
        Vector3::new(2.0, 4.0,  0.0),
        Vector3::new(2.0, 8.0,  0.0),
        Vector3::new(4.0, 0.0,  0.0),
        Vector3::new(4.0, 4.0,  0.0),
        Vector3::new(4.0, 8.0,  3.0),
        Vector3::new(6.0, 0.0,  0.0),
        Vector3::new(6.0, 4.0, -3.0),
        Vector3::new(6.0, 8.0,  0.0),
    ];
    let knot_vector_u = vec![
        0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0,
    ];
    let knot_vector_v = vec![
        0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
    ];

    let surface = bezier::BSplineSurface::new(
        control_points,
        knot_vector_u,
        knot_vector_v,
        3, // degree_u
        2, // degree_v
        4, // size_u
        3, // size_v
    );

    let point = surface.evaluate_single(0.1, 0.2);
    println!("{}", point);

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for i in 0u16..100 {
        for j in 0u16..100 {
            let u = i as f32 / 100.0;
            let v = j as f32 / 100.0;
            let coord = surface.evaluate_single(u, v);
            vertices.push(Point3::from(coord));

            if i > 0 && j > 0 {
                let idx_1 = i * 100 + j;
                let idx_2 = (i - 1) * 100 + j;
                let idx_3 = i * 100 + (j - 1);
                indices.push(Point3::new(idx_1, idx_2, idx_3))
            }

            if i < 99 && j < 99 {
                let idx_1 = i * 100 + j;
                let idx_2 = (i + 1) * 100 + j;
                let idx_3 = i * 100 + (j + 1);
                indices.push(Point3::new(idx_1, idx_2, idx_3))
            }
        }
    }

    let mesh = Rc::new(RefCell::new(Mesh::new(
        vertices, indices, None, None, false,
    )));

    let mut window = Window::new("[ME 625] Bezier Demo");
    window.set_light(Light::StickToCamera);

    let mut c = window.add_mesh(mesh, Vector3::new(1.0, 1.0, 1.0));
    c.set_color(1.0, 0.0, 0.0);
    c.enable_backface_culling(false);

    while window.render() {
        sleep(30);
    }
}

fn main() {
    bezier_tests();
}
