extern crate nalgebra as na;

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

pub struct Link {
    pub name: String,
    pub mass: f32,
}
