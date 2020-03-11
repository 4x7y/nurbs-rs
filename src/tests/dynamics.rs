use crate::math::*;
use crate::robotics::*;

fn create_robot() -> RobotModel {
    RobotModel::new()
}

#[test]
fn test_composite_rigid_body_algorithm() {
    let robot = create_robot();
    // let q = VectorDf::zeros(5);
    // let mm = robot.ximm(&q);

    assert_eq!(true, true);
}

#[test]
fn test_rne() {
    let mut robot: RobotModel = create_robot();
    robot.nv = 3;
    robot.adjacent_tform = vec![
        Matrix4f::new(
            1., 0., 0., 0.,
            0., 1., 0., 0.,
            0., 0., 1., 0.089159,
            0., 0., 0., 1.,
        ),
        Matrix4f::new(
            0., 0., 1., 0.28,
            0., 1., 0., 0.13585,
           -1., 0., 0., 0.,
            0., 0., 0., 1.,
        ),
        Matrix4f::new(
            1., 0., 0., 0.,
            0., 1., 0.,-0.1197,
            0., 0., 1., 0.395,
            0., 0., 0., 1.,
        ),
        Matrix4f::new(
            1., 0., 0., 0.,
            0., 1., 0., 0.,
            0., 0., 1., 0.14225,
            0., 0., 0., 1.,
        ),
    ];
    robot.spatial_inertia = vec![
        Vector6f::new(0.0102670, 0.0102670, 0.0066600, 3.700, 3.700, 3.700),
        Vector6f::new(0.2268900, 0.2268900, 0.0151074, 8.393, 8.393, 8.393),
        Vector6f::new(0.0494433, 0.0494433, 0.0040950, 2.275, 2.275, 2.275),
    ];
    robot.screw = vec![
        Vector6f::new(1.0, 0.0, 1.0,    0.0, 1.0,   0.0),
        Vector6f::new(0.0, 1.0, 0.0, -0.089, 0.0,   0.0),
        Vector6f::new(0.0, 1.0, 0.0, -0.089, 0.0, 0.425),
    ];

    let mut state = RobotState::new(3, 3, 3, 3, 4);

    state.qpos = VectorDf::from_row_slice(&[0.1, 0.1, 0.1]);
    state.qvel = VectorDf::from_row_slice(&[0.1, 0.2, 0.3]);
    state.qacc = VectorDf::from_row_slice(&[2.0, 1.5, 1.0]);

    let xfrc_tip = Vector6f::new(1., 1., 1., 1., 1., 1.);
    let tau = rne(&robot, &state, xfrc_tip);
    assert_relative_eq!(tau,
        VectorDf::from_row_slice(&[74.69616155, -33.06766016, -3.23057314]),
        epsilon=1.0e-1);
}