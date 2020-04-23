#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(non_snake_case)]
#![cfg_attr(debug_assertions, allow(dead_code, unused_imports))]

extern crate nalgebra as na;
extern crate kiss3d;

pub mod math;
pub mod geometry;
pub mod robotics;
pub mod utils;
pub mod control;
pub mod simulation;
pub mod ccd;            // convex collision detection

#[macro_use]
extern crate approx;
#[macro_use]
extern crate failure;
#[macro_use]
extern crate prettytable;
extern crate byteorder;

extern crate log4rs;
use log::{error, info, warn};

#[cfg(test)]
mod tests;