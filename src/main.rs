extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::light::Light;
use kiss3d::resource::Mesh;
use kiss3d::text::Font;
use kiss3d::window::Window;
use na::{DMatrix};
use na::{Point2, Point3, UnitQuaternion, Vector3};
use std::cell::RefCell;
use std::rc::Rc;
mod geometry;

use geometry::bezier;

fn bezier_tests() {
    let bernstein = bezier::bernstein(0, 5, 0.3);
    println!("{}", bernstein);

    let all_bernstein = bezier::all_bernstein(5, 0.3);
    println!("{}", all_bernstein);

    let control_points = DMatrix::from_row_slice(3, 4, &[
            1.5, -4.7, 0.2, -5.9,
            5.8, 7.81, 0.8, 17.2,
            1.4, -2.1, 6.6, -4.1,
        ],
    );
    let point = bezier::point_on_bezier_curve(control_points, 3, 0.1);
    println!("{}", point);
}

fn kiss3d_tests() {    
    let mut window = Window::new("Kiss3d: custom_mesh");

    let a = Point3::new(-1.0, -1.0, 0.0);
    let b = Point3::new(1.0, -1.0, 0.0);
    let c = Point3::new(0.0, 1.0, 0.0);

    let vertices = vec![a, b, c];
    let indices = vec![Point3::new(0u16, 1, 2)];

    let mesh = Rc::new(RefCell::new(Mesh::new(
        vertices, indices, None, None, false,
    )));
    let mut c = window.add_mesh(mesh, Vector3::new(1.0, 1.0, 1.0));

    c.set_color(1.0, 0.0, 0.0);
    c.enable_backface_culling(false);

    window.set_light(Light::StickToCamera);

    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);

    let font = Font::default();

    while window.render() {
        c.prepend_to_local_rotation(&rot);

        window.draw_text(
            "Hello birds!",
            &Point2::origin(),
            120.0,
            &font,
            &Point3::new(0.0, 1.0, 1.0),
        );

        let ascii = " !\"#$%&'`()*+,-_./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^abcdefghijklmnopqrstuvwxyz{|}~";

        window.draw_text(
            ascii,
            &Point2::new(0.0, 120.0),
            60.0,
            &font,
            &Point3::new(1.0, 1.0, 0.0),
        );
    }
}

fn main() {
    bezier_tests();
}
