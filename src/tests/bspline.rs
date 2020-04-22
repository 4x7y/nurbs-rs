// #[test]
// fn test_bspline() {
//     let control_points = vec![
//         Vector3f::new(-25.0, -25.0, -10.0),
//         Vector3f::new(-25.0, -15.0, -5.0),
//         Vector3f::new(-25.0, -5.0, 0.0),
//         Vector3f::new(-25.0, 5.0, 0.0),
//         Vector3f::new(-25.0, 15.0, -5.0),
//         Vector3f::new(-25.0, 25.0, -10.0),
//         Vector3f::new(-15.0, -25.0, -8.0),
//         Vector3f::new(-15.0, -15.0, -4.0),
//         Vector3f::new(-15.0, -5.0, -4.0),
//         Vector3f::new(-15.0, 5.0, -4.0),
//         Vector3f::new(-15.0, 15.0, -4.0),
//         Vector3f::new(-15.0, 25.0, -8.0),
//         Vector3f::new(-5.0, -25.0, -5.0),
//         Vector3f::new(-5.0, -15.0, -3.0),
//         Vector3f::new(-5.0, -5.0, -8.0),
//         Vector3f::new(-5.0, 5.0, -8.0),
//         Vector3f::new(-5.0, 15.0, -3.0),
//         Vector3f::new(-5.0, 25.0, -5.0),
//         Vector3f::new(5.0, -25.0, -3.0),
//         Vector3f::new(5.0, -15.0, -2.0),
//         Vector3f::new(5.0, -5.0, -8.0),
//         Vector3f::new(5.0, 5.0, -8.0),
//         Vector3f::new(5.0, 15.0, -2.0),
//         Vector3f::new(5.0, 25.0, -3.0),
//         Vector3f::new(15.0, -25.0, -8.0),
//         Vector3f::new(15.0, -15.0, -4.0),
//         Vector3f::new(15.0, -5.0, -4.0),
//         Vector3f::new(15.0, 5.0, -4.0),
//         Vector3f::new(15.0, 15.0, -4.0),
//         Vector3f::new(15.0, 25.0, -8.0),
//         Vector3f::new(25.0, -25.0, -10.0),
//         Vector3f::new(25.0, -15.0, -5.0),
//         Vector3f::new(25.0, -5.0, 2.0),
//         Vector3f::new(25.0, 5.0, 2.0),
//         Vector3f::new(25.0, 15.0, -5.0),
//         Vector3f::new(25.0, 25.0, -10.0),
//     ];
//     let knot_vector_u = vec![
//         0.0, 0.0, 0.0, 0.0, 0.33, 0.66, 1.0, 1.0, 1.0, 1.0
//     ];
//     let knot_vector_v = vec![
//         0.0, 0.0, 0.0, 0.0, 0.33, 0.66, 1.0, 1.0, 1.0, 1.0
//     ];
//
//     let surface = bspline::BSplineSurface::new(
//         control_points,
//         knot_vector_u,
//         knot_vector_v,
//         3,               // degree_u
//         3,               // degree_v
//         6,               // size_u
//         6,               // size_v
//     );
//     let mut surface_ins = surface.clone();
//
//     surface_ins.insert_knot((Some(0.3), None), (2, 0));
//     let pt = surface_ins.evaluate_single(0.3, 0.4);
//     println!("{:.4}", pt);
//
//     let mut window = Window::new("[ME 625] B-Spline Surface Demo");
//     window.set_light(Light::StickToCamera);
//
//     let mesh_ins = surface_ins.get_mesh();
//     return mesh_ins;
// }
