#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![cfg_attr(debug_assertions, allow(dead_code, unused_imports))]

extern crate nalgebra as na;

use crobot::geometry::{bspline, DoubleEdgeList, nurbs};
use kiss3d::light::Light;
use kiss3d::window::Window;
use kiss3d::resource::Mesh;
use na::{Point3, Vector3, VectorN, U4, Vector};
use std::{thread, time};
use std::cell::RefCell;
use std::rc::Rc;
use log::{info, error};
use crobot::utils::JointSpaceTrajectory;
use crobot::math::*;
use crobot::robotics::{RigidBodyTree, trvec2tform};
use crobot::utils::{read_stl, load_mesh};
use std::fs::OpenOptions;
use crobot::simulation::sim_model::SimScene;
use crobot::geometry::*;
use crobot::ccd::*;

fn sleep(millis: u64) {
    let duration = time::Duration::from_millis(millis);
    thread::sleep(duration);
}

fn nurbs_surf_1() -> NurbsSurface {
    let control_points = vec![
        Vector3f::new(-2.5, -2.5,  -1.0),
        Vector3f::new(-2.5, -1.5,  -0.5),
        Vector3f::new(-2.5, -0.5,   0.0),
        Vector3f::new(-2.5,  0.5,   0.0),
        Vector3f::new(-2.5,  1.5,  -0.5),
        Vector3f::new(-2.5,  2.5,  -1.0),
        Vector3f::new(-1.5, -2.5,  -0.8),
        Vector3f::new(-1.5, -1.5,  -0.4),
        Vector3f::new(-1.5, -0.5,  -0.4),
        Vector3f::new(-1.5,  0.5,  -0.4),
        Vector3f::new(-1.5,  1.5,  -0.4),
        Vector3f::new(-1.5,  2.5,  -0.8),
        Vector3f::new(-0.5, -2.5,  -0.5),
        Vector3f::new(-0.5, -1.5,  -0.3),
        Vector3f::new(-0.5, -0.5,  -0.8),
        Vector3f::new(-0.5,  0.5,  -0.8),
        Vector3f::new(-0.5,  1.5,  -0.3),
        Vector3f::new(-0.5,  2.5,  -0.5),
        Vector3f::new( 0.5, -2.5,  -0.3),
        Vector3f::new( 0.5, -1.5,  -0.2),
        Vector3f::new( 0.5, -0.5,  -0.8),
        Vector3f::new( 0.5,  0.5,  -0.8),
        Vector3f::new( 0.5,  1.5,  -0.2),
        Vector3f::new( 0.5,  2.5,  -0.3),
        Vector3f::new( 1.5, -2.5,  -0.8),
        Vector3f::new( 1.5, -1.5,  -0.4),
        Vector3f::new( 1.5, -0.5,  -0.4),
        Vector3f::new( 1.5,  0.5,  -0.4),
        Vector3f::new( 1.5,  1.5,  -0.4),
        Vector3f::new( 1.5,  2.5,  -0.8),
        Vector3f::new( 2.5, -2.5,  -1.0),
        Vector3f::new( 2.5, -1.5,  -0.5),
        Vector3f::new( 2.5, -0.5,   0.0),
        Vector3f::new( 2.5,  0.5,   0.0),
        Vector3f::new( 2.5,  1.5,  -0.5),
        Vector3f::new( 2.5,  2.5, -1.0),
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

    nurbs::NurbsSurface::new(
        control_points,
        knot_vector_u,
        knot_vector_v,
        3,               // degree_u
        3,               // degree_v
        6,               // size_u
        6,               // size_v
        weight,
    )
}

fn nurbs_surf_2() -> NurbsSurface {

    let frac1sqrt2 = std::f64::consts::FRAC_1_SQRT_2 as Scalar;

    let control_points = vec![
        Vector3f::new(1.0,0.0,0.0),
        Vector3f::new(frac1sqrt2,frac1sqrt2,0.0),
        Vector3f::new(0.0,1.0,0.0),
        Vector3f::new(-frac1sqrt2,frac1sqrt2,0.0),
        Vector3f::new(-1.0,0.0,0.0),

        Vector3f::new(1.0,0.0,1.0),
        Vector3f::new(frac1sqrt2,frac1sqrt2,frac1sqrt2),
        Vector3f::new(0.0,1.0,1.0),
        Vector3f::new(-frac1sqrt2,frac1sqrt2,frac1sqrt2),
        Vector3f::new(-1.0,0.0,1.0),
    ];

    let weight = vec![
        1., frac1sqrt2, 1., frac1sqrt2, 1.,
        1., frac1sqrt2, 1., frac1sqrt2, 1.];
    let knotvec_u = vec![0., 0., 1., 1.];
    let knotvec_v = vec![0., 0., 0., 0.5, 0.5, 1., 1., 1.];
    let degree_u = 1;
    let degree_v = 2;

    nurbs::NurbsSurface::new(
        control_points,
        knotvec_u,
        knotvec_v,
        degree_u,        // degree_u
        degree_v,        // degree_v
        2,               // size_u
        5,               // size_v
        weight,
    )
}

fn nurbs_surf_3() -> NurbsSurface {

    let frac1sqrt2 = std::f64::consts::FRAC_1_SQRT_2 as Scalar;
    let control_points = vec![
        Vector3f::new(1.0,0.0,0.0),
        Vector3f::new(frac1sqrt2,frac1sqrt2,0.0),
        Vector3f::new(0.0,1.0,0.0),
        Vector3f::new(-frac1sqrt2,frac1sqrt2,0.0),
        Vector3f::new(-1.0,0.0,0.0),
        Vector3f::new(-frac1sqrt2,-frac1sqrt2,0.0),
        Vector3f::new(0.0,-1.0,0.0),
        Vector3f::new(frac1sqrt2,-frac1sqrt2,0.0),
        Vector3f::new(1.0,0.0,0.0),
        
        Vector3f::new(1.0,0.0,1.0),
        Vector3f::new(frac1sqrt2,frac1sqrt2,frac1sqrt2),
        Vector3f::new(0.0,1.0,1.0),
        Vector3f::new(-frac1sqrt2,frac1sqrt2,frac1sqrt2),
        Vector3f::new(-1.0,0.0,1.0),
        Vector3f::new(-frac1sqrt2,-frac1sqrt2,frac1sqrt2),
        Vector3f::new(0.0,-1.0,1.0),
        Vector3f::new(frac1sqrt2,-frac1sqrt2,frac1sqrt2),
        Vector3f::new(1.0,0.0,1.0)
    ];

    let weight = vec![
        1.0, frac1sqrt2, 1.0, frac1sqrt2, 1.0, frac1sqrt2, 1.0, frac1sqrt2, 1.0,
        1.0, frac1sqrt2, 1.0, frac1sqrt2, 1.0, frac1sqrt2, 1.0, frac1sqrt2, 1.0];
    let knotvec_u = vec![0., 0., 1., 1.];
    let knotvec_v = vec![0., 0., 0., 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1., 1., 1.];
    let degree_u = 1;
    let degree_v = 2;

    nurbs::NurbsSurface::new(
        control_points,
        knotvec_u,
        knotvec_v,
        degree_u,        // degree_u
        degree_v,        // degree_v
        2,               // size_u
        9,               // size_v
        weight,
    )
}

fn nurbs_surf_4() -> NurbsSurface {

    let control_points = vec![
        Vector3f::new(-2.5, -2.5, -8.0),
        Vector3f::new(-2.5, -1.5, -8.0),
        Vector3f::new(-2.5, -0.5, -8.0),
        Vector3f::new(-2.5,  0.5, -8.0),
        Vector3f::new(-2.5,  1.5, -8.0),
        Vector3f::new(-2.5,  2.5, -8.0),

        Vector3f::new(-1.5, -2.5, -8.0),
        Vector3f::new(-1.5, -1.5, -7.6),
        Vector3f::new(-1.5, -0.5, -5.6),
        Vector3f::new(-1.5,  0.5, -5.6),
        Vector3f::new(-1.5,  1.5, -7.6),
        Vector3f::new(-1.5,  2.5, -8.0),

        Vector3f::new(-0.5, -2.5, -8.0),
        Vector3f::new(-0.5, -1.5, -6.0),
        Vector3f::new(-0.5, -0.5, -4.2),
        Vector3f::new(-0.5,  0.5, -4.2),
        Vector3f::new(-0.5,  1.5, -6.0),
        Vector3f::new(-0.5,  2.5, -8.0),

        Vector3f::new( 0.5, -2.5, -8.0),
        Vector3f::new( 0.5, -1.5, -6.0),
        Vector3f::new( 0.5, -0.5, -4.2),
        Vector3f::new( 0.5,  0.5, -4.2),
        Vector3f::new( 0.5,  1.5, -6.0),
        Vector3f::new( 0.5,  2.5, -8.0),

        Vector3f::new( 1.5, -2.5, -8.0),
        Vector3f::new( 1.5, -1.5, -7.6),
        Vector3f::new( 1.5, -0.5, -5.6),
        Vector3f::new( 1.5,  0.5, -5.6),
        Vector3f::new( 1.5,  1.5, -7.6),
        Vector3f::new( 1.5,  2.5, -8.0),

        Vector3f::new( 2.5, -2.5, -8.0),
        Vector3f::new( 2.5, -1.5, -8.0),
        Vector3f::new( 2.5, -0.5, -8.0),
        Vector3f::new( 2.5,  0.5, -8.0),
        Vector3f::new( 2.5,  1.5, -8.0),
        Vector3f::new( 2.5,  2.5, -8.0),
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

    nurbs::NurbsSurface::new(
        control_points,
        knot_vector_u,
        knot_vector_v,
        3,               // degree_u
        3,               // degree_v
        6,               // size_u
        6,               // size_v
        weight,
    )
}

fn nurbs_test() {

    let surf1 = nurbs_surf_1();
    let surf2 = nurbs_surf_4();


    // let surf = nurbs::NurbsSurface::new(
    //     control_points_2,
    //     knot_vector_u,
    //     knot_vector_v,
    //     3,               // degree_u
    //     3,               // degree_v
    //     6,               // size_u
    //     6,               // size_v
    //     weight,
    // );
    //
    // let (surf1, surf2) = surf.split_surface_u(0.5);
    //
    // let obb1 = OBB::from(&surf1);
    // let obb2 = OBB::from(&surf2);
    // println!("{}", obb1.intersects(&obb2, 0.001));

    // let tree1 = OBBTree::from_nurbs_surface(&surf1, 0);
    // let tree2 = OBBTree::from_nurbs_surface(&surf2, 0);
    // println!("collide = {}", tree1.intersect(&tree2, 0.001));


    let mut scene = SimScene::new("[ME 625] B-Spline Surface Demo");
    // obb1.register_scene(&mut scene);
    // obb2.register_scene(&mut scene);


    let tree2 = OBBTree::from_nurbs_surface(&surf2, 4);
    let obbs = tree2.collect_base_obb();
    // for obb in &mut obbs {
    //     obb.register_scene(&mut scene);
    // }

    // let mesh = surf1.get_mesh();
    // let mut h1 = scene.window.add_mesh(mesh, Vector3::new(1.0, 1.0, 1.0));
    // h1.set_color(1.0, 0.0, 0.0);
    // h1.enable_backface_culling(false);


    let mesh = surf2.get_mesh();
    let mut h2 = scene.window.add_mesh(mesh, Vector3::new(1.0, 1.0, 1.0));
    h2.set_color(0.0, 0.0, 1.0);
    h2.enable_backface_culling(false);

    // let vel = Vector3f::new(0., 0., 0.01);
    // let tform = trvec2tform(vel.clone());
    // let trans = na::Translation3::new(
    //     vel[0] as f32, vel[1] as f32, vel[2] as f32);


    while scene.render() {
        sleep(30);
        // obb1.render();
        // obb2.render();

        // for obb in &mut obbs {
        //     obb.render();
        // }

        // surf2.transform(&tform);
        // let tree2 = OBBTree::from_nurbs_surface(&surf2, 10);
        // if tree1.intersect(&tree2, 0.005) {
        //     h2.set_color(0.0, 1.0, 0.0);
        // } else {
        //     h2.set_color(0.0, 0.0, 1.0);
        // }
        // h2.append_translation(&trans);
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

struct Box {
    pos: Vector3f,
    x: Scalar,
    y: Scalar,
    z: Scalar,
    rotm: Matrix3f,
}

impl Box {
    fn new(pos: Vector3f, dim: Vec<Scalar>) -> Self {
        Box {
            pos: pos,
            x: dim[0],
            y: dim[1],
            z: dim[2],
            rotm: Matrix3f::identity(),
        }
    }
}

impl CCDObject for Box {
    fn center(&self) -> Vector3f {
        self.pos.clone_owned()
    }

    fn support(&self, dir: &Vector3f) -> Vector3f {

        let dir_local = self.rotm.try_inverse().unwrap() * dir;
        // compute support point in specified direction
        let vec_local = Vector3f::new(
            dir_local[0].signum() * self.x * 0.5,
            dir_local[1].signum() * self.y * 0.5,
            dir_local[2].signum() * self.z * 0.5
        );
        let vec = self.rotm * &vec_local + &self.pos;

        return vec;
    }
}

use crobot::ccd::mpr_penetration;

fn ccd_test() {
    let ccd = CCDCriteria {
        max_iterations: 100,
        epa_tolerance: 0.0001,
        mpr_tolerance: 0.0001,
        dist_tolerance: 1e-6
    };

    let obj1 = Box::new(Vector3f::zeros(), vec![0.2, 0.2, 0.2]);
    let obj2 = Box::new(Vector3f::new(0.0, 0.0, 0.0), vec![0.2, 0.2, 0.2]);
    let mut info = CCDResult::new();
    let res = mpr_penetration(&obj1, &obj2, &ccd, &mut info);
    println!("result = {}", res);
    println!("{:?}", &info);
}

fn main() {
    ccd_test();
    // nurbs_test();
    // sim_test();
}
