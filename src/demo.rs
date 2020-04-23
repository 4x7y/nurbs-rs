#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![cfg_attr(debug_assertions, allow(dead_code, unused_imports))]

extern crate nalgebra as na;

use crobot::geometry::{bspline, DoubleEdgeList, nurbs};
use kiss3d::light::Light;
use kiss3d::window::Window;
use kiss3d::resource::Mesh;
use na::{Point3, Vector3, VectorN, U4};
use std::{thread, time};
use std::cell::RefCell;
use std::rc::Rc;
use log::{info, error};
use crobot::utils::JointSpaceTrajectory;
use crobot::math::*;
use crobot::robotics::RigidBodyTree;
use crobot::utils::{read_stl, load_mesh};
use std::fs::OpenOptions;
use crobot::simulation::sim_model::SimScene;
use crobot::geometry::*;
use crobot::ccd::*;

fn sleep(millis: u64) {
    let duration = time::Duration::from_millis(millis);
    thread::sleep(duration);
}

fn nurbs_test() {

    let control_points = vec![
        Vector3f::new(-25.0, -25.0, -10.0),
        Vector3f::new(-25.0, -15.0,  -5.0),
        Vector3f::new(-25.0,  -5.0,   0.0),
        Vector3f::new(-25.0,   5.0,   0.0),
        Vector3f::new(-25.0,  15.0,  -5.0),
        Vector3f::new(-25.0,  25.0, -10.0),
        Vector3f::new(-15.0, -25.0,  -8.0),
        Vector3f::new(-15.0, -15.0,  -4.0),
        Vector3f::new(-15.0,  -5.0,  -4.0),
        Vector3f::new(-15.0,   5.0,  -4.0),
        Vector3f::new(-15.0,  15.0,  -4.0),
        Vector3f::new(-15.0,  25.0,  -8.0),
        Vector3f::new( -5.0, -25.0,  -5.0),
        Vector3f::new( -5.0, -15.0,  -3.0),
        Vector3f::new( -5.0,  -5.0,  -8.0),
        Vector3f::new( -5.0,   5.0,  -8.0),
        Vector3f::new( -5.0,  15.0,  -3.0),
        Vector3f::new( -5.0,  25.0,  -5.0),
        Vector3f::new(  5.0, -25.0,  -3.0),
        Vector3f::new(  5.0, -15.0,  -2.0),
        Vector3f::new(  5.0,  -5.0,  -8.0),
        Vector3f::new(  5.0,   5.0,  -8.0),
        Vector3f::new(  5.0,  15.0,  -2.0),
        Vector3f::new(  5.0,  25.0,  -3.0),
        Vector3f::new( 15.0, -25.0,  -8.0),
        Vector3f::new( 15.0, -15.0,  -4.0),
        Vector3f::new( 15.0,  -5.0,  -4.0),
        Vector3f::new( 15.0,   5.0,  -4.0),
        Vector3f::new( 15.0,  15.0,  -4.0),
        Vector3f::new( 15.0,  25.0,  -8.0),
        Vector3f::new( 25.0, -25.0, -10.0),
        Vector3f::new( 25.0, -15.0,  -5.0),
        Vector3f::new( 25.0,  -5.0,   0.0),
        Vector3f::new( 25.0,   5.0,   0.0),
        Vector3f::new( 25.0,  15.0,  -5.0),
        Vector3f::new( 25.0,  25.0, -10.0),
    ];
    let knot_vector_u = vec![
        0.0, 0.0, 0.0, 0.0, 0.33, 0.66, 1.0, 1.0, 1.0, 1.0
    ];
    let knot_vector_v = vec![
        0.0, 0.0, 0.0, 0.0, 0.33, 0.66, 1.0, 1.0, 1.0, 1.0
    ];
    let mut weight = vec![1.; 36];
    for i in 18..36 {
        weight[i] = 20.;
    }

    let surface = nurbs::NurbsSurface::new(
        control_points,
        knot_vector_u,
        knot_vector_v,
        3,               // degree_u
        3,               // degree_v
        6,               // size_u
        6,               // size_v
        weight,
    );

    let mut bbox = OBB::from(&surface);
    println!("{}", bbox);

    let mut scene = SimScene::new("[ME 625] B-Spline Surface Demo");
    bbox.register_scene(&mut scene);

    let mesh = surface.get_mesh();
    let mut d = scene.window.add_mesh(mesh, Vector3::new(1.0, 1.0, 1.0));
    d.set_color(1.0, 0.0, 0.0);
    d.enable_backface_culling(false);

    while scene.render() {
        sleep(30);
        bbox.render();
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

fn sim_test() {
    log4rs::init_file("resource/log4rs.yaml", Default::default()).unwrap();
    info!("booting up...");

    let mut model: RigidBodyTree = RigidBodyTree::from_urdf_file("resource/sample.urdf").unwrap();
    info!("\n{}", model);

    let mut scene = SimScene::new("Demo");
    model.register_scene(&mut scene);
    // let mut h = scene.window.add_mesh(bspline_test(), Vector3::new(1.0, 1.0, 1.0));
    // h.set_color(1.0, 0.0, 0.0);
    // h.enable_backface_culling(false);

    let qpos_home = model.home_configuration();

    let mass_matrix = model.mass_matrix(&qpos_home);
    info!("\n{:.4}", mass_matrix);

    let res = model.mass_matrix_internal(&qpos_home);
    info!("\n{:?}", res.1);

    let fext = vec![Vector6f::zeros(); model.num_body()];
    let torq = model.inverse_dynamics(&qpos_home, &qpos_home, &qpos_home, &fext);
    info!("\n{:.8}", torq);

    let torq = VectorDf::repeat(model.num_dof(), 1.);
    let qacc = model.forward_dynamics_crb(&qpos_home, &qpos_home, &torq, &fext);
    info!("qacc [CRB] = \n{:.4}", qacc / 100000.0);
    let qacc = model.forward_dynamics_ab(&qpos_home, &qpos_home, &torq, &fext);
    info!("qacc [AB] = \n{:.4}", qacc / 100000.0);

    info!("{:?}", model.joint_names_non_fixed());

    let mut qpos = qpos_home;
    let mut qpos_inc = qpos.clone_owned();
    qpos_inc[0] = 0.001;
    qpos_inc[1] = 0.001;
    qpos_inc[2] = -0.001;
    let mut qvel = VectorDf::zeros(model.num_dof());
    let torq = VectorDf::zeros(model.num_dof());

    let dt = 0.002;
    let mut t = 0.0;
    while scene.window.render() {
        let qacc = model.forward_dynamics_ab(&qpos, &qvel, &torq, &fext);
        qvel = qvel + &qacc * dt;
        qpos = qpos + &qvel * dt;
        model.render(&qpos);
        t += dt;
    }
}


fn main() {
    nurbs_test();
    // sim_test();
}
