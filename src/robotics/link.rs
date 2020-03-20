use crate::math::*;
use std::fmt;
use failure::_core::fmt::{Formatter, Error};
use crate::robotics::{skew, spatial_inertia};
use std::fmt::Display;

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
            inertial: Inertial::from_mass(0.),
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

impl Display for Geometry {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            Geometry::Box { depth, width, height } => {
                write!(f, "Box (w: {}, h:{}, d: {})", width, height, depth)
            },
            Geometry::Cylinder { radius, length } => {
                write!(f, "Cylinder (r: {}, len:{})", radius, length)
            },
            Geometry::Capsule { radius, length } => {
                write!(f, "Capsule (r: {}, len:{})", radius, length)
            },
            Geometry::Sphere { radius } => {
                write!(f, "Sphere (r: {})", radius)
            },
            Geometry::Mesh { filename, scale } => {
                write!(f, "Mesh (file: {}, scale: [{:.1}, {:.1}, {:.1}])",
                       filename, scale[0], scale[1], scale[2])},
        }
    }
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
    pub bvec_com: Vector3f,              // center of mass
    pub mass: Scalar,                    // mass
    pub inertia_body: Matrix3f,          // 3x3 inertia tensor of the rigid body relative to body frame
    pub inertia_com: Matrix3f,           // 3x3 inertia tensor of the rigid body relative to inertia frame
    pub spatial_inertia: Matrix6f,       // 6x6 spatial inertia matrix
}

impl Display for Inertial {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        write!(f, "mass: {:.2}, com: [{:.3} {:.3} {:.3}], \
                   \nbody_inertia: [{:.6} {:.6} {:.6} {:.6} {:.6} {:.6}]",
               self.mass,
               self.bvec_com[0], self.bvec_com[1], self.bvec_com[2],
               self.inertia_body[(0, 0)], self.inertia_body[(1, 1)], self.inertia_body[(2, 2)],
               self.inertia_body[(0, 1)], self.inertia_body[(0, 2)], self.inertia_body[(1, 2)])
    }
}

impl Inertial {
    pub fn from_mass(mass: Scalar) -> Self {
        Self {
            bvec_com: Vector3f::identity(),
            mass: mass,
            inertia_body: Matrix3f::identity(),
            inertia_com: Matrix3f::identity(),
            spatial_inertia: Matrix6f::identity(),
        }
    }
    pub fn new(bvec_com: Vector3f, mass: Scalar,
               inertia_com: Matrix3f, inertia_body: Matrix3f) -> Self {
        Self {
            bvec_com: bvec_com,
            mass: mass,
            inertia_body: inertia_body,
            inertia_com: inertia_com,
            spatial_inertia: spatial_inertia(mass, bvec_com, inertia_com),
        }
    }
    pub fn set_com(&mut self, origin: Vector3f) {
        self.bvec_com = origin;
    }
    pub fn com(&self) -> &Vector3f {
        &self.bvec_com
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