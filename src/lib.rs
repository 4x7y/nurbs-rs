#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![cfg_attr(debug_assertions, allow(dead_code, unused_imports))]

extern crate nalgebra as na;
extern crate kiss3d;

pub mod math;
pub mod geometry;
pub mod robotics;
pub mod utils;
pub mod control;

#[macro_use]
extern crate approx;

#[cfg(test)]
mod tests;