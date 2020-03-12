use urdf_rs;

use na::{self, Isometry3, Matrix3, RealField};
use std::collections::HashMap;
use std::path::Path;
use crate::math::*;
use crate::robotics::*;

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
fn axis_from(array3: [f64; 3]) -> na::Unit<Vector3f> {
    na::Unit::<_>::new_normalize(na::Vector3::new(
        na::convert(array3[0]),
        na::convert(array3[1]),
        na::convert(array3[2]),
    ))
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
        Inertial::new(
            isometry_from(&urdf_inertial.origin),
            na::convert(urdf_inertial.mass.value),
            Matrix3::new(
                na::convert(i.ixx),
                na::convert(i.ixy),
                na::convert(i.ixz),
                na::convert(i.ixy),
                na::convert(i.iyy),
                na::convert(i.iyz),
                na::convert(i.ixz),
                na::convert(i.iyz),
                na::convert(i.izz),
            ),
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
                filename,
                scale: na::Vector3::new(
                    na::convert(scale[0]),
                    na::convert(scale[1]),
                    na::convert(scale[2]),
                ),
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
                _ => JointType::Fixed,
            })
            .limits(limit)
            .rotation(quaternion_from(&joint.origin.rpy))
            .translation(translation_from(&joint.origin.xyz))
            .finalize()
    }
}

// impl<'a, T> From<&'a urdf_rs::Robot> for Chain<T>
//     where
//         T: RealField,
// {
//     fn from(robot: &urdf_rs::Robot) -> Self {
//         let mut ref_nodes = Vec::new();
//         let mut child_link_name_to_node = HashMap::new();
//         let mut joint_name_to_node = HashMap::new();
//         let mut parent_link_name_to_node = HashMap::<&String, Vec<Node<T>>>::new();
//         let root_node = JointBuilder::<T>::new().name(ROOT_JOINT_NAME).into_node();
//         for j in &robot.joints {
//             let node = Node::<T>::new(j.into());
//             child_link_name_to_node.insert(&j.child.link, node.clone());
//             if parent_link_name_to_node.get(&j.parent.link).is_some() {
//                 parent_link_name_to_node
//                     .get_mut(&j.parent.link)
//                     .unwrap()
//                     .push(node.clone());
//             } else {
//                 parent_link_name_to_node.insert(&j.parent.link, vec![node.clone()]);
//             }
//             ref_nodes.push(node.clone());
//             joint_name_to_node.insert(j.name.clone(), node);
//         }
//         for l in &robot.links {
//             info!("link={}", l.name);
//             if let Some(parent_node) = child_link_name_to_node.get_mut(&l.name) {
//                 if let Some(child_nodes) = parent_link_name_to_node.get(&l.name) {
//                     for child_node in child_nodes.iter() {
//                         info!("set parent = {}, child = {}", parent_node, child_node);
//                         child_node.set_parent(parent_node);
//                     }
//                 }
//                 parent_node.set_link(Some(l.clone().into()));
//             } else {
//                 info!("root={}", l.name);
//                 root_node.set_link(Some(l.clone().into()));
//             }
//         }
//         // add mimics
//         for j in &robot.joints {
//             if j.mimic.tmp != "" {
//                 debug!("mimic found for {}", j.mimic.tmp);
//                 let child = joint_name_to_node[&j.name].clone();
//                 let parent = joint_name_to_node
//                     .get(&j.mimic.tmp)
//                     .unwrap_or_else(|| panic!("{} not found, mimic not found", &j.mimic.tmp));
//                 child.set_mimic_parent(parent, (&j.mimic).into());
//             }
//         }
//         // set root as parent of root tmp nodes
//         let root_nodes = ref_nodes
//             .iter()
//             .filter(|ref_node| ref_node.parent().is_none());
//         for rjn in root_nodes {
//             info!("set parent = {}, child = {}", root_node, rjn);
//             rjn.set_parent(&root_node);
//         }
//         Chain::from_root(root_node)
//     }
// }
//
// impl<T> From<urdf_rs::Robot> for Chain<T>
//     where
//         T: RealField,
// {
//     fn from(robot: urdf_rs::Robot) -> Self {
//         Self::from(&robot)
//     }
// }
//
// impl<T> Chain<T>
//     where
//         T: RealField,
// {
//     pub fn from_urdf_file<P>(path: P) -> Result<Self, urdf_rs::UrdfError>
//         where
//             P: AsRef<Path>,
//     {
//         Ok(urdf_rs::read_file(path)?.into())
//     }
// }
//
// /// Useful function to deal about 'Links' of URDF
// ///
// /// `k` deals only `Joint`s of URDF. But links is connected
// /// to tmp always, it is easily find which tmp is the
// /// parent of the link.
// ///
// /// # Examples
// ///
// /// ```
// /// extern crate urdf_rs;
// /// extern crate k;
// ///
// /// let urdf_robot = urdf_rs::read_file("urdf/sample.urdf").unwrap();
// /// let map = k::urdf::link_to_joint_map(&urdf_robot);
// /// assert_eq!(map.get("root_body").unwrap(), k::urdf::ROOT_JOINT_NAME);
// /// assert_eq!(map.get("r_wrist2").unwrap(), "r_wrist_pitch");
// /// assert!(map.get("no_exist_link").is_none());
// /// ```
// pub fn link_to_joint_map(urdf_robot: &urdf_rs::Robot) -> HashMap<String, String> {
//     let mut map = HashMap::new();
//     for j in &urdf_robot.joints {
//         map.insert(j.child.link.to_owned(), j.name.to_owned());
//     }
//     for l in &urdf_robot.links {
//         if map.get(&l.name).is_none() {
//             map.insert(l.name.to_owned(), ROOT_JOINT_NAME.to_owned());
//         }
//     }
//     map
// }
//
// pub fn joint_to_link_map(urdf_robot: &urdf_rs::Robot) -> HashMap<String, String> {
//     let mut map = HashMap::new();
//     for j in &urdf_robot.joints {
//         map.insert(j.name.to_owned(), j.child.link.to_owned());
//     }
//     for l in &urdf_robot.links {
//         if map.get(&l.name).is_none() {
//             map.insert(ROOT_JOINT_NAME.to_owned(), l.name.to_owned());
//         }
//     }
//     map
// }
