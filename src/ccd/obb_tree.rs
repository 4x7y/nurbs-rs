use crate::geometry::NurbsSurface;
use crate::ccd::OBB;
use core::fmt;
use std::fmt::Formatter;
use crate::math::*;

#[derive(Clone)]
pub struct OBBTree {
    left:  Option<Box<OBBTree>>,
    right: Option<Box<OBBTree>>,
    obb:   Option<Box<OBB>>,
    level: usize,
}

impl OBBTree {

    pub fn new() -> Self {
        OBBTree {
            left:  None,
            right: None,
            obb:   None,
            level: 0,
        }
    }


    pub fn from_nurbs_surface(surf: &NurbsSurface, level: usize) -> Self {
        if level == 0 {
            OBBTree {
                left:  None,
                right: None,
                obb:   Some(Box::new(OBB::from(surf))),
                level: level,
            }
        } else {
            let (surf1, surf2) = if level % 2 == 0 {
                surf.split_surface_u(0.5)
            } else {
                surf.split_surface_v(0.5)
            };

            OBBTree {
                left:  Some(Box::new(Self::from_nurbs_surface(&surf1, level-1))),
                right: Some(Box::new(Self::from_nurbs_surface(&surf2, level-1))),
                obb:   Some(Box::new(OBB::from(surf))),
                level: level,
            }
        }
    }

    fn collect_base_obb_helper(&self, list: &mut Vec<OBB>) {
        if self.level == 0 {
            if let Some(obb) = &self.obb {
                list.push(obb.as_ref().clone());
            }
        } else {
            self.left.as_ref().unwrap().collect_base_obb_helper(list);
            self.right.as_ref().unwrap().collect_base_obb_helper(list);
        }
    }

    pub fn collect_base_obb(&self) -> Vec<OBB> {
        let mut res = Vec::new();
        self.collect_base_obb_helper(&mut res);
        return res;
    }

    pub fn intersect(&self, other: &OBBTree, epsilon: Scalar) -> bool {
        if let Some(a) = &self.obb {
            if let Some(b) = &other.obb {
                if a.intersects(b, epsilon) {
                    let mut no_rec = true;
                    let r1 = if let Some(left1) = self.left.as_ref() {
                        let r1 = if let Some(left2) = other.left.as_ref() {
                            no_rec = false;
                            left1.intersect(left2, epsilon)
                        } else {
                            false
                        };

                        let r2 = if let Some(right2) = other.right.as_ref() {
                            no_rec = false;
                            left1.intersect(right2, epsilon)
                        } else {
                            false
                        };

                        r1 || r2
                    } else {
                        false
                    };

                    let r2 = if let Some(right1) = self.right.as_ref() {
                        let r1 = if let Some(left2) = other.left.as_ref() {
                            no_rec = false;
                            right1.intersect(left2, epsilon)
                        } else {
                            false
                        };

                        let r2 = if let Some(right2) = other.right.as_ref() {
                            no_rec = false;
                            right1.intersect(right2, epsilon)
                        } else {
                            false
                        };

                        r1 || r2
                    } else {
                        false
                    };

                    return r1 || r2 || no_rec
                }
            }
        }

        return false;
    }
}


// impl fmt::Display for OBBTree {
//
//     fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
//         write!(f, "({}, {})", self.level, self.r)
//     }
// }
