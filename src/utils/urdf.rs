use urdf_rs;

use na::{self, Isometry3, Matrix3, RealField};
use std::collections::HashMap;
use std::path::Path;
use crate::math::*;
use crate::robotics::*;
use crate::utils::*;
use kiss3d::resource::Mesh;

impl<'a> From<&'a urdf_rs::Color> for Color {
    fn from(urdf_color: &urdf_rs::Color) -> Self {
        Color {
            r: na::convert(urdf_color.rgba[0]),
            g: na::convert(urdf_color.rgba[1]),
            b: na::convert(urdf_color.rgba[2]),
            a: na::convert(urdf_color.rgba[3]),
        }
    }
}

impl From<urdf_rs::Color> for Color {
    fn from(urdf_color: urdf_rs::Color) -> Self {
        (&urdf_color).into()
    }
}

impl From<urdf_rs::Texture> for Texture {
    fn from(urdf_texture: urdf_rs::Texture) -> Self {
        Texture {
            filename: urdf_texture.filename,
        }
    }
}

impl From<urdf_rs::Material> for Material {
    fn from(urdf_material: urdf_rs::Material) -> Self {
        Material {
            name: urdf_material.name,
            color: urdf_material.color.into(),
            texture: urdf_material.texture.into(),
        }
    }
}

/// Returns nalgebra::Translation3 from f64 array
pub fn translation_from(array3: &[f64; 3]) -> na::Translation3<Scalar> {
    na::convert(na::Translation3::new(array3[0], array3[1], array3[2]))
}

/// Returns nalgebra::Unit<nalgebra::Vector3> from f64 array
fn axis_from(array3: [f64; 3]) -> Matrix3Df {
    Matrix3Df::from_row_slice(&[
        na::convert(array3[0]),
        na::convert(array3[1]),
        na::convert(array3[2]),
    ])
}

/// Returns nalgebra::UnitQuaternion from f64 array
pub fn quaternion_from(array3: &[f64; 3]) -> na::UnitQuaternion<Scalar> {
    na::convert(na::UnitQuaternion::from_euler_angles(
        array3[0], array3[1], array3[2],
    ))
}

pub fn isometry_from(origin_element: &urdf_rs::Pose) -> Isometry3f {
    Isometry3::from_parts(
        translation_from(&origin_element.xyz),
        quaternion_from(&origin_element.rpy),
    )
}

impl From<urdf_rs::Inertial> for Inertial {
    fn from(urdf_inertial: urdf_rs::Inertial) -> Self {
        let i = urdf_inertial.inertia;
        let mass = na::convert(urdf_inertial.mass.value);
        let inertia_com = Matrix3f::new(
            na::convert(i.ixx),
            na::convert(i.ixy),
            na::convert(i.ixz),
            na::convert(i.ixy),
            na::convert(i.iyy),
            na::convert(i.iyz),
            na::convert(i.ixz),
            na::convert(i.iyz),
            na::convert(i.izz),
        );
        let bvec_com = Vector3f::from_column_slice(&urdf_inertial.origin.xyz);
        let rpy = Vector3f::from_column_slice(&urdf_inertial.origin.rpy);
        let ypr = Vector3f::new(rpy[2], rpy[1], rpy[0]);
        let ypr = EulerAngle::ZYX(ypr);
        let rotm_com2body = eul2rotm(ypr);
        let inertia_body = inertia_com2body_with_rot(
            mass, bvec_com, rotm_com2body, inertia_com
        );

        Inertial::new(
            bvec_com,
            mass,
            inertia_com,
            inertia_body,
        )
    }
}

impl From<urdf_rs::Visual> for Visual {
    fn from(urdf_visual: urdf_rs::Visual) -> Self {
        Visual::new(
            urdf_visual.name,
            isometry_from(&urdf_visual.origin),
            urdf_visual.geometry.into(),
            urdf_visual.material.into(),
        )
    }
}

impl From<urdf_rs::Collision> for Collision {
    fn from(urdf_collision: urdf_rs::Collision) -> Self {
        Collision::new(
            urdf_collision.name,
            isometry_from(&urdf_collision.origin),
            urdf_collision.geometry.into(),
        )
    }
}

impl From<urdf_rs::Geometry> for Geometry {
    fn from(urdf_geometry: urdf_rs::Geometry) -> Self {
        match urdf_geometry {
            urdf_rs::Geometry::Box { size } => Geometry::Box {
                depth: na::convert(size[0]),
                width: na::convert(size[1]),
                height: na::convert(size[2]),
            },
            urdf_rs::Geometry::Cylinder { radius, length } => Geometry::Cylinder {
                radius: na::convert(radius),
                length: na::convert(length),
            },
            urdf_rs::Geometry::Sphere { radius } => Geometry::Sphere {
                radius: na::convert(radius),
            },
            urdf_rs::Geometry::Mesh { filename, scale } => Geometry::Mesh {
                filename: filename.clone(),
                scale: na::Vector3::new(
                    na::convert(scale[0]),
                    na::convert(scale[1]),
                    na::convert(scale[2]),
                ),
                mesh: load_mesh(filename),
            },
            urdf_rs::Geometry::Capsule { radius, length } => Geometry::Capsule {
                radius: na::convert(radius),
                length: na::convert(length),
            }
        }
    }
}

impl From<urdf_rs::Link> for Link {
    fn from(urdf_link: urdf_rs::Link) -> Self {
        Link {
            name:       urdf_link.name.to_string(),
            inertial:   urdf_link.inertial.into(),
            visuals:    urdf_link.visual.into_iter().map(|v| v.into()).collect(),
            collisions: urdf_link.collision.into_iter().map(|v| v.into()).collect(),
        }
    }
}

impl<'a> From<&'a urdf_rs::Mimic> for Mimic {
    fn from(urdf_mimic: &urdf_rs::Mimic) -> Self {
        Mimic::new(
            na::convert(urdf_mimic.multiplier),
            na::convert(urdf_mimic.offset),
        )
    }
}

impl<'a> From<&'a urdf_rs::Joint> for Joint {
    fn from(joint: &urdf_rs::Joint) -> Joint {
        let limit = if (joint.limit.upper - joint.limit.lower) == 0.0 {
            None
        } else {
            Some(Range::new(
                na::convert(joint.limit.lower),
                na::convert(joint.limit.upper),
            ))
        };

        JointBuilder::new()
            .name(&joint.name)
            .joint_type(match joint.joint_type {
                urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous => {
                    JointType::Revolute {
                        axis: axis_from(joint.axis.xyz),
                    }
                }
                urdf_rs::JointType::Prismatic => JointType::Prismatic {
                    axis: axis_from(joint.axis.xyz),
                },
                urdf_rs::JointType::Fixed => JointType::Fixed,
                _ => panic!("Joint types floating and planar are not supported."),
            })
            .limits(limit)
            .safety_controller(joint.safety_controller.soft_upper_limit,
                               joint.safety_controller.soft_lower_limit,
                               joint.safety_controller.k_position,
                               joint.safety_controller.k_velocity)
            .dynamics(joint.dynamics.damping,
                      joint.dynamics.friction,
                      joint.limit.effort)
            .tform_jnt2parent(Vector3f::from_column_slice(&joint.origin.xyz),
                              Vector3f::from_column_slice(&joint.origin.rpy))
            .finalize()
    }
}
