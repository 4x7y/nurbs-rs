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
use log::{info, error};
use crobot::utils::trajectory::JointSpaceTrajectory;
use crobot::math::*;
use crobot::robotics::RigidBodyTree;
use crobot::utils::{read_stl, load_mesh};
use std::fs::OpenOptions;

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
            let u = i as Scalar / 100.0;
            let v = j as Scalar / 100.0;
            let coord = surface.evaluate_single(u, v);
            vertices.push(Point3::new(
                coord[0] as f32, coord[1] as f32, coord[2] as f32));

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

fn dcel_test() {
    let mut dcel = DoubleEdgeList::<Vector2f, Scalar, String>::new();

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

    dcel.initialize(&vertices, &edges, &face);

    let vec = VectorNf::<U4>::zeros();
    let vech = vec.to_homogeneous();
    println!("{}", vec);
    println!("{}", vech);
}



fn main() {
    log4rs::init_file("resource/log4rs.yaml", Default::default()).unwrap();
    info!("booting up...");

    let model: RigidBodyTree = RigidBodyTree::from_urdf_file("resource/sample.urdf").unwrap();
    println!("{}", model);

    let joint = model.get_joint("ee_fixed_joint");
    println!("{}", joint);

    let body = model.get_body("forearm_link");
    println!("{}", body);

    let mut meshes = Vec::new();
    meshes.push(load_mesh("resource/meshes/universal/base.stl"));
    meshes.push(load_mesh("resource/meshes/universal/forearm.stl"));
    // meshes.push(load_mesh("resource/meshes/universal/driver.stl"));
    // meshes.push(load_mesh("resource/meshes/universal/link_0.STL"));
    // meshes.push(load_mesh("resource/meshes/universal/link_1.STL"));
    // meshes.push(load_mesh("resource/meshes/universal/link_2.STL"));
    // meshes.push(load_mesh("resource/meshes/universal/link_3.STL"));
    // meshes.push(load_mesh("resource/meshes/universal/shoulder.stl"));
    // meshes.push(load_mesh("resource/meshes/universal/upperarm.stl"));
    // meshes.push(load_mesh("resource/meshes/universal/wrist1.stl"));
    // meshes.push(load_mesh("resource/meshes/universal/wrist2.stl"));
    // meshes.push(load_mesh("resource/meshes/universal/wrist3.stl"));

    // let mut window = Window::new("Mesh Rendering");
    // window.set_light(Light::StickToCamera);
    //
    // for mesh in meshes {
    //     let mut c = window.add_mesh(mesh, Vector3::new(1.0, 1.0, 1.0));
    //     c.set_color(0.5, 0.5, 0.5);
    //     c.enable_backface_culling(true);
    // }
    //
    // while window.render() {
    //     sleep(30);
    // }
}
