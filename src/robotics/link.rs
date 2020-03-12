use crate::math::*;
use std::fmt;
use failure::_core::fmt::{Formatter, Error};

/// abstract geom
pub struct Geom {
    // type info
    pub geom_type: i32,                 // geom type (mjtGeom)
    pub data_id: i32,                   // mesh, hfield or plane id; -1: none
    pub obj_type: i32,                  // object type; mjOBJ_UNKNOWN for decor
    pub obj_id: i32,                    // object id; -1 for decor
    pub category: i32,                  // visual category
    pub tex_id: i32,                    // texture id; -1: no texture
    pub tex_uniform: i32,               // uniform cube mapping
    pub tex_coord: i32,                 // mesh geom has texture coordinates
    pub seg_id: i32,                    // segmentation id; -1: not shown

    // transparency rendering (set internally)
    pub cam_dist: f32,                  // distance to camera (used by sorter)
    pub model_rbound: f32,              // geom rbound from model, 0 if not model geom
    pub transparent: f32,               // treat geom as transparent
}

#[derive(Debug, Clone)]
pub struct Link {
    pub name: String,
    pub inertial: Inertial,
    pub visuals: Vec<Visual>,
    pub collisions: Vec<Collision>,
}

impl Default for Link {
    fn default() -> Self {
        Self {
            name: "".to_owned(),
            inertial: Inertial::new(Isometry3f::identity(), 0., Matrix3f::identity()),
            visuals: Vec::new(),
            collisions: Vec::new(),
        }
    }
}

impl fmt::Display for Link {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.name)
    }
}

#[derive(Debug, Clone)]
pub enum Geometry {
    Box      { depth: Scalar, width: Scalar, height: Scalar },
    Cylinder { radius: Scalar, length: Scalar },
    Capsule  { radius: Scalar, length: Scalar },
    Sphere   { radius: Scalar },
    Mesh     { filename: String, scale: Vector3f },
}

#[derive(Debug, Default, Clone)]
pub struct Color {
    pub r: Scalar,
    pub g: Scalar,
    pub b: Scalar,
    pub a: Scalar,
}

#[derive(Debug, Clone)]
pub struct Texture {
    pub filename: String,
}

impl Default for Texture {
    fn default() -> Texture {
        Texture {
            filename: "".to_string(),
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Material {
    pub name: String,
    pub color: Color,
    pub texture: Texture,
}

#[derive(Debug, Clone)]
pub struct Inertial {
    origin: Isometry3f,
    pub mass: Scalar,
    pub inertia: Matrix3f,
}


impl Inertial {
    pub fn from_mass(mass: Scalar) -> Self {
        Self {
            origin: Isometry3f::identity(),
            mass: mass,
            inertia: Matrix3f::identity(),
        }
    }
    pub fn new(origin: Isometry3f, mass: Scalar, inertia: Matrix3f) -> Self {
        Self {
            origin: origin,
            mass: mass,
            inertia: inertia,
        }
    }
    pub fn set_origin(&mut self, origin: Isometry3f) {
        self.origin = origin;
    }
    pub fn origin(&self) -> &Isometry3f {
        &self.origin
    }
}

#[derive(Debug, Clone)]
pub struct Visual {
    pub name: String,
    origin: Isometry3f,
    pub geometry: Geometry,
    pub material: Material,
}

impl Visual {
    
    pub fn new(name: String, origin: Isometry3f, geometry: Geometry, material: Material) -> Self {
        Self {
            name: name,
            origin: origin,
            geometry: geometry,
            material: material,
        }
    }
    
    pub fn set_origin(&mut self, origin: Isometry3f) {
        self.origin = origin;
    }

    pub fn origin(&self) -> &Isometry3f {
        &self.origin
    }
}

#[derive(Debug, Clone)]
pub struct Collision {
    pub name: String,
    origin: Isometry3f,
    pub geometry: Geometry,
}

impl Collision {

    pub fn new(name: String, origin: Isometry3f, geometry: Geometry) -> Self {
        Self {
            name: name,
            origin: origin,
            geometry: geometry,
        }
    }
    pub fn set_origin(&mut self, origin: Isometry3f) {
        self.origin = origin;
    }

    pub fn origin(&self) -> &Isometry3f {
        &self.origin
    }
}