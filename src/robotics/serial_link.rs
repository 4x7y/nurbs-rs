extern crate nalgebra as na;
use na::Vector3;

pub struct SerialLink {
    pub name: String,
    pub gravity: Vector3<f32>,
}
