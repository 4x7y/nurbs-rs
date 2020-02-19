use crate::math::matrix::*;
use crate::robotics::dynamics::*;
use crate::robotics::robot::Robot;

fn create_robot() -> Robot {
    Robot::new()
}

#[test]
fn test_composite_rigid_body_algorithm() {
    let robot = create_robot();
    let q = VectorDf::zeros(5);
let mm = robot.ximm(&q);

println!("{}", mm);
}