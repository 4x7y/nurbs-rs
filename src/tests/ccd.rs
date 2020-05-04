use crate::ccd::*;
use crate::math::{Vector3f, Matrix3f, Scalar};
use crate::utils::rotm2quat;
use crate::robotics::axang2rotm;
use std::f64::consts::{FRAC_PI_3, FRAC_PI_4, PI};

#[test]
fn test_box_cylinder() {
    let mut obj_box = Box {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(0.5, 1., 1.5),
    };

    let mut obj_cylinder = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.4,
        height: 0.7,
    };

    let ccd = CCDCriteria::default();
    let mut res = CCDResult::new();

    // test 1
    obj_cylinder.pos = Vector3f::new(0.1, 0.0, 0.0);
    let is_intersect = mpr_penetration(&obj_box, &obj_cylinder, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 2
    obj_cylinder.pos = Vector3f::new(0.6, 0.0, 0.0);
    let is_intersect = mpr_penetration(&obj_box, &obj_cylinder, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 3
    obj_cylinder.pos = Vector3f::new(0.6, 0.6, 0.0);
    let is_intersect = mpr_penetration(&obj_box, &obj_cylinder, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 4
    obj_cylinder.pos = Vector3f::new(0.6, 0.6, 0.5);
    let is_intersect = mpr_penetration(&obj_box, &obj_cylinder, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 5
    obj_cylinder.pos = Vector3f::new(0.6, 0.0, 0.5);
    obj_cylinder.rotm = axang2rotm(Vector3f::new(0., 1., 0.), FRAC_PI_3 as Scalar);
    let is_intersect = mpr_penetration(&obj_box, &obj_cylinder, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 6
    obj_cylinder.pos = Vector3f::new(0.6, 0.0, 0.5);
    obj_cylinder.rotm = axang2rotm(Vector3f::new(0.67, 1.1, 0.12), FRAC_PI_4 as Scalar);
    let is_intersect = mpr_penetration(&obj_box, &obj_cylinder, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 7
    obj_cylinder.pos = Vector3f::new(0.6, 0.0, 0.5);
    obj_cylinder.rotm = axang2rotm(Vector3f::new(-0.1, 2.2, -1.), PI as Scalar / 5.);
    obj_box.pos = Vector3f::new(0.6, 0., 0.5);
    obj_box.rotm = axang2rotm(Vector3f::new(1., 1., 0.), -FRAC_PI_4);
    let is_intersect = mpr_penetration(&obj_box, &obj_cylinder, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 8
    obj_cylinder.pos = Vector3f::new(0.6, 0.0, 0.5);
    obj_cylinder.rotm = axang2rotm(Vector3f::new(-0.1, 2.2, -1.), PI as Scalar / 5.);
    obj_box.pos = Vector3f::new(0.9, 0.8, 0.5);
    obj_box.rotm = axang2rotm(Vector3f::new(1., 1., 0.), -FRAC_PI_4);
    let is_intersect = mpr_penetration(&obj_box, &obj_cylinder, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);
}

#[test]
fn test_sphere_sphere_aligned_x() {
    let mut sphere_1 = Sphere {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.35
    };

    let sphere_2 = Sphere {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.5
    };

    sphere_1.pos = Vector3f::new(-5., 0., 0.);
    for i in 0..100 {
        let ccd = CCDCriteria::default();
        let res = mpr_intersect(&sphere_1, &sphere_2, &ccd);

        if i < 42 || i > 58 {
            assert!(!res);
        } else {
            assert!(res);
        }

        sphere_1.pos[0] += 0.1;
    }
}

#[test]
fn test_sphere_sphere_aligned_y() {
    let mut sphere_1 = Sphere {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.35
    };

    let sphere_2 = Sphere {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.5
    };

    sphere_1.pos = Vector3f::new(0., -5., 0.);
    for i in 0..100 {
        let ccd = CCDCriteria::default();
        let res = mpr_intersect(&sphere_1, &sphere_2, &ccd);

        if i < 42 || i > 58 {
            assert!(!res);
        } else {
            assert!(res);
        }

        sphere_1.pos[1] += 0.1;
    }
}

#[test]
fn test_sphere_sphere_aligned_z() {
    let mut sphere_1 = Sphere {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.35
    };

    let sphere_2 = Sphere {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.5
    };

    sphere_1.pos = Vector3f::new(0., 0., -5.);
    for i in 0..100 {
        let ccd = CCDCriteria::default();
        let res = mpr_intersect(&sphere_1, &sphere_2, &ccd);

        if i < 42 || i > 58 {
            assert!(!res);
        } else {
            assert!(res);
        }

        sphere_1.pos[2] += 0.1;
    }
}


#[test]
fn test_cylinder_cylinder_aligned_x() {
    let mut cylinder_1 = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.35,
        height: 0.5,
    };

    let cylinder_2 = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.5,
        height: 1.0,
    };

    cylinder_1.pos = Vector3f::new(-5., 0., 0.);
    for i in 0..100 {
        let ccd = CCDCriteria::default();
        let res = mpr_intersect(&cylinder_1, &cylinder_2, &ccd);

        if i < 42 || i > 58 {
            assert!(!res);
        } else {
            assert!(res);
        }

        cylinder_1.pos[0] += 0.1;
    }
}

#[test]
fn test_cylinder_cylinder_aligned_y() {
    let mut cylinder_1 = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.35,
        height: 0.5,
    };

    let cylinder_2 = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.5,
        height: 1.0,
    };

    cylinder_1.pos = Vector3f::new(0., -5., 0.);
    for i in 0..100 {
        let ccd = CCDCriteria::default();
        let res = mpr_intersect(&cylinder_1, &cylinder_2, &ccd);

        if i < 42 || i > 58 {
            assert!(!res);
        } else {
            assert!(res);
        }

        cylinder_1.pos[1] += 0.1;
    }
}

#[test]
fn test_cylinder_cylinder_aligned_z() {
    let mut cylinder_1 = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.35,
        height: 0.5,
    };

    let cylinder_2 = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.5,
        height: 1.0,
    };

    cylinder_1.pos = Vector3f::new(0., 0., -5.);
    for i in 0..100 {
        let ccd = CCDCriteria::default();
        let res = mpr_intersect(&cylinder_1, &cylinder_2, &ccd);

        if i < 43 || i > 57 {
            assert!(!res);
        } else {
            assert!(res);
        }

        cylinder_1.pos[2] += 0.1;
    }
}

#[test]
fn test_cylinder_cylinder_penetration() {
    let mut cylinder_1 = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.35,
        height: 0.5,
    };

    let mut cylinder_2 = Cylinder {
        pos: Vector3f::zeros(),
        rotm: Matrix3f::identity(),
        radius: 0.5,
        height: 1.0,
    };

    let mut res = CCDResult::new();
    let ccd = CCDCriteria::default();

    // test 1
    cylinder_2.pos = Vector3f::new(0., 0., 0.3);
    let is_intersect = mpr_penetration(&cylinder_1, &cylinder_2, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 2
    cylinder_1.pos = Vector3f::new(0.3, 0.1, 0.1);
    let is_intersect = mpr_penetration(&cylinder_1, &cylinder_2, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 3
    cylinder_2.rotm = axang2rotm(Vector3f::new(0., 1., 1.), FRAC_PI_4);
    cylinder_2.pos  = Vector3f::new(0., 0., 0.);
    let is_intersect = mpr_penetration(&cylinder_1, &cylinder_2, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 4
    cylinder_2.rotm = axang2rotm(Vector3f::new(0., 1., 1.), FRAC_PI_4);
    cylinder_2.pos  = Vector3f::new( -0.2, 0.7, 0.2);
    let is_intersect = mpr_penetration(&cylinder_1, &cylinder_2, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 5
    cylinder_2.rotm = axang2rotm(Vector3f::new(0.567, 1.2, 1.), FRAC_PI_4);
    cylinder_2.pos  = Vector3f::new(0.6, -0.7, 0.2);
    let is_intersect = mpr_penetration(&cylinder_1, &cylinder_2, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);

    // test 6
    cylinder_2.rotm = axang2rotm(Vector3f::new(-4.567, 1.2, 0.), FRAC_PI_3);
    cylinder_2.pos  = Vector3f::new(0.6, -0.7, 0.2);
    let is_intersect = mpr_penetration(&cylinder_1, &cylinder_2, &ccd, &mut res);
    assert!(is_intersect);
    println!("is_intersect = {}, \t {:?}", is_intersect, res);
}

#[test]
fn test_box_box_aligned_x() {
    let mut box_1 = Box {
        pos: Vector3f::new(-5., 0., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(1., 2., 1.),
    };

    let box_2 = Box {
        pos: Vector3f::new(0., 0., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(2., 1., 2.),
    };

    let ccd = CCDCriteria::default();

    for i in 0..100 {
        let res = mpr_intersect(&box_1, &box_2, &ccd);
        if i < 35 || i > 65 {
            assert!(!res);
        } else if i != 35 && i != 65 {
            assert!(res);
        }

        box_1.pos[0] += 0.1;
    }
}

#[test]
fn test_box_box_aligned_x_small() {
    let mut box_1 = Box {
        pos: Vector3f::new(-0.5, 0., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(0.1, 0.2, 0.1),
    };

    let box_2 = Box {
        pos: Vector3f::new(0., 0., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(0.2, 0.1, 0.2),
    };

    let ccd = CCDCriteria::default();

    for i in 0..100 {
        let res = mpr_intersect(&box_1, &box_2, &ccd);
        if i < 35 || i > 65 {
            assert!(!res);
        } else if i != 35 && i != 65 {
            assert!(res);
        }

        box_1.pos[0] += 0.01;
    }
}

#[test]
fn test_box_box_aligned_x_tiny() {
    let mut box_1 = Box {
        pos: Vector3f::new(-0.05, 0., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(0.01, 0.02, 0.01),
    };

    let box_2 = Box {
        pos: Vector3f::new(0., 0., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(0.02, 0.01, 0.02),
    };

    let ccd = CCDCriteria::default();

    for i in 0..100 {
        let res = mpr_intersect(&box_1, &box_2, &ccd);
        if i < 35 || i > 65 {
            assert!(!res);
        } else if i != 35 && i != 65 {
            assert!(res);
        }

        box_1.pos[0] += 0.001;
    }
}

#[test]
fn test_box_box_aligned_y() {
    let mut box_1 = Box {
        pos: Vector3f::new(0., -5., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(1., 2., 1.),
    };

    let box_2 = Box {
        pos: Vector3f::new(0., 0., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(2., 1., 2.),
    };

    let ccd = CCDCriteria::default();

    for i in 0..100 {
        let res = mpr_intersect(&box_1, &box_2, &ccd);
        if i < 35 || i > 65 {
            assert!(!res);
        } else if i != 35 && i != 65 {
            assert!(res);
        }

        box_1.pos[1] += 0.1;
    }
}

#[test]
fn test_box_box_aligned_z() {
    let mut box_1 = Box {
        pos: Vector3f::new(0., 0., -5.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(1., 2., 1.),
    };

    let box_2 = Box {
        pos: Vector3f::new(0., 0., 0.),
        rotm: Matrix3f::identity(),
        dim: Vector3f::new(2., 1., 2.),
    };

    let ccd = CCDCriteria::default();

    for i in 0..100 {
        let res = mpr_intersect(&box_1, &box_2, &ccd);
        if i < 35 || i > 65 {
            assert!(!res);
        } else if i != 35 && i != 65 {
            assert!(res);
        }

        box_1.pos[2] += 0.1;
    }
}