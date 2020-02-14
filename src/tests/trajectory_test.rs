use crate::math::matrix::Vector5f;
use crate::utils::trajectory::JointSpaceTrajectory;
use na::U5;

#[test]
fn from_trap_vel() {

    let wpts = vec![
        Vector5f::new( 0.0, 45.0,  15.0, 90.0, 45.0),
        Vector5f::new(90.0, 45.0, -45.0, 15.0, 90.0),
    ];
    JointSpaceTrajectory::<U5>::from_trap_vel(&wpts, 150);

    println!("hello world");
}