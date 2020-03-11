#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![cfg_attr(debug_assertions, allow(dead_code, unused_imports))]

extern crate nalgebra as na;

use crobot::geometry::{bspline, DoubleEdgeList};
use kiss3d::light::Light;
use kiss3d::window::Window;
use kiss3d::resource::Mesh;
use na::{Point3, Vector3, VectorN, U4};
use std::{thread, time};
use std::cell::RefCell;
use std::rc::Rc;
use crobot::utils::trajectory::JointSpaceTrajectory;
use crobot::math::*;


fn sleep(millis: u64) {
    let duration = time::Duration::from_millis(millis);
    thread::sleep(duration);
}

fn bspline_test() {

    let control_points = vec![
        Vector3f::new(0.0, 0.0,  0.0),
        Vector3f::new(0.0, 4.0,  0.0),
        Vector3f::new(0.0, 8.0, -3.0),
        Vector3f::new(2.0, 0.0,  6.0),
        Vector3f::new(2.0, 4.0,  0.0),
        Vector3f::new(2.0, 8.0,  0.0),
        Vector3f::new(4.0, 0.0,  0.0),
        Vector3f::new(4.0, 4.0,  0.0),
        Vector3f::new(4.0, 8.0,  3.0),
        Vector3f::new(6.0, 0.0,  0.0),
        Vector3f::new(6.0, 4.0, -3.0),
        Vector3f::new(6.0, 8.0,  0.0),
    ];
    let knot_vector_u = vec![
        0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0,
    ];
    let knot_vector_v = vec![
        0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
    ];

    let surface = bspline::BSplineSurface::new(
        control_points,
        knot_vector_u,
        knot_vector_v,
        3,               // degree_u
        2,               // degree_v
        4,                 // size_u
        3,                 // size_v
    );

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

    let mut window = Window::new("[ME 625] B-Spline Surface Demo");
    window.set_light(Light::StickToCamera);

    let mut c = window.add_mesh(mesh, Vector3::new(1.0, 1.0, 1.0));
    c.set_color(1.0, 0.0, 0.0);
    c.enable_backface_culling(false);

    while window.render() {
        sleep(30);
    }
}

fn main() {
    let mut dcel = DoubleEdgeList::<Vector2f, f32, String>::new();

    let vertices = vec![
        Vector2f::new(400., 150.),
        Vector2f::new(350., 350.),
        Vector2f::new(300., 250.),
    ];

    let edges = vec![
        1., -1.,
        2., -2.,
        3., -3.,
    ];

    let face = String::from("Face");

    // dcel.initialize(vertices, edges, face);

    

    
    let vec = VectorNf::<U4>::zeros();
    let vech = vec.to_homogeneous();
    println!("{}", vec);
    println!("{}", vech);

}
