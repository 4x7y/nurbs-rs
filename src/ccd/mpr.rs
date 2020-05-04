use crate::math::{Vector3f, Scalar};
use crate::ccd::{CCDObject, CCDCriteria};
use crate::ccd::helper::*;
use std::cmp::min;

#[derive(Debug, Clone)]
pub struct CCDResult {
    pub depth: Scalar,
    pub dir:   Vector3f,
    pub pos:   Vector3f,
}

impl CCDResult {
    pub fn new() -> Self {
        CCDResult {
            depth: 0.0,
            dir: Vector3f::zeros(),
            pos: Vector3f::zeros(),
        }
    }
}

/// Returns true if two given objects intersect - MPR algorithm is used.
pub fn mpr_intersect(obj1: &dyn CCDObject,
                     obj2: &dyn CCDObject,
                     ccd: &CCDCriteria) -> bool {
    let mut portal = CCDSimplex::new();

    // Phase 1: Portal discovery - find portal that intersects with origin
    // ray (ray from center of Minkowski diff to origin of coordinates)
    let res = discover_portal(obj1, obj2, ccd, &mut portal);
    if res < 0 {
        return false;
    }
    if res > 0 {
        return true;
    }

    // Phase 2: Portal refinement
    let res = refine_portal(obj1, obj2, ccd, &mut portal);
    return if res == 0 { true } else { false };
}

/// Computes penetration of obj2 into obj1.
/// Depth of penetration, direction and position is returned, i.e. if obj2
/// is translated by computed depth in resulting direction obj1 and obj2
/// would have touching contact. Position is point in global coordinates
/// where force should take a place.
///
/// Minkowski Portal Refinement algorithm is used (MPR, a.k.a. XenoCollide,
/// see Game Programming Gem 7).
///
/// Returns 0 if obj1 and obj2 intersect, otherwise -1 is returned.
pub fn mpr_penetration(obj1: &dyn CCDObject,
                       obj2: &dyn CCDObject,
                       ccd: &CCDCriteria,
                       info: &mut CCDResult) -> bool {
    let mut portal = CCDSimplex::new();
    let mut res: i32;

    // Phase 1: Portal discovery
    res = discover_portal(obj1, obj2, ccd, &mut portal);
    if res < 0 {
        // Origin isn't inside portal - no collision.
        return false;

    } else if res == 1 {
        // Touching contact on portal's v1.
        find_penetration_touch(obj1, obj2, ccd, &mut portal, info);

    } else if res == 2 {
        // Origin lies on v0-v1 segment.
        find_penetration_segment(obj1, obj2, ccd, &mut portal, info);

    } else if res == 0 {
        // Phase 2: Portal refinement
        res = refine_portal(obj1, obj2, ccd, &mut portal);
        if res < 0 {
            return false;
        }

        // Phase 3. Penetration info
        find_penetration(obj1, obj2, ccd, &mut portal, info);
    }

    return true;
}

/// Finds origin (center) of Minkowski difference (actually it can be any
/// interior point of Minkowski difference.
fn find_origin(obj1: &dyn CCDObject,
               obj2: &dyn CCDObject,
               ccd: &CCDCriteria,
               center: &mut CCDSupport) {
    center.v1 = obj1.center();
    center.v2 = obj2.center();
    center.v  = center.v1 - center.v2;
}

/// Discovers initial portal - that is tetrahedron that intersects with
/// origin ray (ray from center of Minkowski diff to (0,0,0).
///
///  Returns -1 if already recognized that origin is outside Minkowski
///  portal.
///  Returns 1 if origin lies on v1 of simplex (only v0 and v1 are present
///  in simplex).
///  Returns 2 if origin lies on v0-v1 segment.
///  Returns 0 if portal was built.
fn discover_portal(obj1: &dyn CCDObject,
                   obj2: &dyn CCDObject,
                   ccd: &CCDCriteria,
                   portal: &mut CCDSimplex) -> i32 {

    let mut dir: Vector3f;
    let mut va:  Vector3f;
    let mut vb:  Vector3f;

    let origin = Vector3f::zeros();

    let dot: Scalar;
    let mut cont: bool;

    find_origin(obj1, obj2, ccd, portal.point_mut(0));
    portal.set_size(1);


    if vec_eq_approx(&portal.point(0).v, &origin) {
        // Portal's center lies on origin (0,0,0) => we know that objects
        // intersect but we would need to know penetration info.
        // So move center little bit...
        va = Vector3f::new(CCD_EPS * 10., 0., 0.);
        portal.point_mut(0).v += &va;
    }

    // vertex 1 = support in direction of origin
    dir = -1. * portal.point(0).v;
    dir.normalize_mut();
    portal.point_mut(1).initialize(obj1, obj2, &dir, ccd);
    portal.set_size(2);

    // test if origin isn't outside of v1
    dot = portal.point(1).v.dot(&dir);
    if dot < CCD_ZERO {
        return -1;
    }

    // vertex 2
    dir = portal.point(0).v.cross(&portal.point(1).v);
    if is_zero_approx(dir.norm_squared()) {
        if vec_eq_approx(&portal.point(1).v, &origin) {
            // origin lies on v1
            return 1;
        } else {
            // origin lies on v0-v1 segment
            return 2;
        }
    }

    dir.normalize_mut();
    portal.point_mut(2).initialize(obj1, obj2, &dir, ccd);
    let dot = portal.point(2).v.dot(&dir);
    if dot < CCD_ZERO {
        return -1;
    }
    portal.set_size(3);

    // vertex 3 direction
    va  = portal.point(1).v - &portal.point(0).v;
    vb  = portal.point(2).v - &portal.point(0).v;
    dir = va.cross(&vb);
    dir.normalize_mut();

    // it is better to form portal faces to be oriented "outside" origin
    let dot = portal.point(0).v.dot(&dir);
    if dot > CCD_ZERO {
        portal.swap(1, 2);
        dir.scale_mut(-1.);
    }

    while portal.size() < 4 {
        portal.point_mut(3).initialize(obj1, obj2, &dir, ccd);
        let dot = portal.point(3).v.dot(&dir);
        if dot < CCD_ZERO || is_zero_approx(dot) {
            return -1;
        }

        cont = false;

        // test if origin is outside (v1, v0, v3) - set v2 as v3 and
        // continue
        va = portal.cross_self_point_v(1, 3);
        let dot = portal.dot_point_v_with(0, &va);
        if dot < CCD_ZERO && !is_zero_approx(dot) {
            portal.set(2, portal.point(3).clone());
            cont = true;
        }

        if !cont {
            // test if origin is outside (v3, v0, v2) - set v1 as v3 and
            // continue
            va = portal.cross_self_point_v(3, 2);
            let dot = portal.dot_point_v_with(0, &va);
            if dot < CCD_ZERO && !is_zero_approx(dot) {
                portal.set(1, portal.point(3).clone());
                cont = true;
            }
        }

        if cont {
            va  = portal.point(1).v - &portal.point(0).v;
            vb  = portal.point(2).v - &portal.point(0).v;
            dir = va.cross(&vb);
            dir.normalize_mut();
        } else {
            portal.set_size(4);
        }
    }

    return 0;
}


fn find_penetration_touch(obj1: &dyn CCDObject,
                          obj2: &dyn CCDObject,
                          ccd: &CCDCriteria,
                          portal: &mut CCDSimplex,
                          info: &mut CCDResult) {
    info.depth = 0.;
    info.dir   = Vector3f::zeros();
    info.pos   = 0.5 * (portal.point(1).v1 + portal.point(1).v2);
}

fn find_penetration_segment(obj1: &dyn CCDObject,
                            obj2: &dyn CCDObject,
                            ccd: &CCDCriteria,
                            portal: &mut CCDSimplex,
                            info: &mut CCDResult) {

    // Origin lies on v0-v1 segment.
    // Depth is distance to v1, direction also and position must be
    // computed
    info.pos   = 0.5 * (portal.point(1).v1 + portal.point(1).v2);
    info.dir   = portal.point(1).v;
    info.depth = info.dir.norm();
    info.dir.normalize_mut();
}

fn refine_portal(obj1: &dyn CCDObject,
                 obj2: &dyn CCDObject,
                 ccd: &CCDCriteria,
                 portal: &mut CCDSimplex) -> i32 {

    let mut dir;
    let mut v4;


    loop {
        // compute direction outside the portal (from v0 through v1,v2,v3
        // face)
        dir = portal_dir(portal);

        // test if origin is inside the portal
        if portal_encapsulate_origin(portal, &dir) {
            return 0;
        }

        // get next support point
        v4 = CCDSupport::from(obj1, obj2, &dir, ccd);

        // test if v4 can expand portal to contain origin and if portal
        // expanding doesn't reach given tolerance
        if !portal_can_encapsulate_origin(portal, &v4, &dir)
            || portal_reach_tolerance(portal, &v4, &dir, ccd) {
            return -1;
        }

        // v1-v2-v3 triangle must be rearranged to face outside Minkowski
        // difference (direction from v0).
        expand_portal(portal, &v4);
    }
}

fn find_penetration(obj1: &dyn CCDObject,
                    obj2: &dyn CCDObject,
                    ccd: &CCDCriteria,
                    portal: &mut CCDSimplex,
                    info: &mut CCDResult) {

    let mut iter: usize = 0;
    let origin = Vector3f::zeros();

    loop {
        // compute portal direction and obtain next support point
        let dir = portal_dir(portal);
        let v4  = CCDSupport::from(obj1, obj2, &dir, ccd);

        // reached tolerance -> find penetration info
        if portal_reach_tolerance(portal, &v4, &dir, ccd) ||
            iter > ccd.max_iterations {
            let depth = vec_point_triangle_dist_sq(&origin,
                                               &portal.point(1).v,
                                               &portal.point(2).v,
                                               &portal.point(3).v,
                                               &mut info.dir);
            info.depth = depth.sqrt();
            if is_zero_approx(info.depth) {
                // If depth is zero, then we have a touching contact.
                // So following `find_penetration_touch()`, we assign zero to
                // the direction vector (it can actually be anything
                // according to the description of `mpr_penetration`
                // function).
                info.dir = origin;
            } else {
                info.dir.normalize_mut();
            }

            // barycentric coordinates:
            info.pos = find_pos(obj1, obj2, ccd, portal);

            return;
        }

        expand_portal(portal, &v4);

        iter += 1;
    }

}

fn find_pos(obj1: &dyn CCDObject,
            obj2: &dyn CCDObject,
            ccd: &CCDCriteria,
            portal: &mut CCDSimplex) -> Vector3f {


    let mut b: [Scalar; 4] = [0., 0., 0., 0.];
    let dir = portal_dir(portal);

    // use barycentric coordinates of tetrahedron to find origin
    let vec = portal.cross_self_point_v(1, 2);
    b[0]    = portal.dot_point_v_with(3, &vec);
    let vec = portal.cross_self_point_v(3, 2);
    b[1]    = portal.dot_point_v_with(0, &vec);
    let vec = portal.cross_self_point_v(0, 1);
    b[2]    = portal.dot_point_v_with(3, &vec);
    let vec = portal.cross_self_point_v(2, 1);
    b[3]    = portal.dot_point_v_with(0, &vec);

    let mut sum = b[0] + b[1] + b[2] + b[3];
    if is_zero_approx(sum) || sum < CCD_ZERO {
        let vec = portal.cross_self_point_v(2, 3);
        b[1]    = dir.dot(&vec);
        let vec = portal.cross_self_point_v(3, 1);
        b[2]    = dir.dot(&vec);
        let vec = portal.cross_self_point_v(1, 2);
        b[3]    = dir.dot(&vec);

        sum = b[1] + b[2] + b[3];
    }
    let inv =  1. / sum;

    let mut p1 = Vector3f::zeros();
    let mut p2 = Vector3f::zeros();
    for i in 0..4 {
        let mut vec = portal.point(i).v1;
        vec.scale_mut(b[i]);
        p1 += &vec;

        let mut vec = portal.point(i).v2;
        vec.scale_mut(b[i]);
        p2 += &vec;
    }
    p1.scale_mut(inv);
    p2.scale_mut(inv);

    let pos = 0.5 * (p1 + &p2);
    return pos;
}

fn portal_dir(portal: &CCDSimplex) -> Vector3f {
    let v2v1 = portal.point(2).v - &portal.point(1).v;
    let v3v1 = portal.point(3).v - &portal.point(1).v;
    let mut dir = v2v1.cross(&v3v1);
    dir.normalize_mut();
    return dir;
}

fn portal_encapsulate_origin(portal: &CCDSimplex, dir: &Vector3f) -> bool {
    let dot = portal.point(1).v.dot(&dir);
    return is_zero_approx(dot) || dot > CCD_ZERO;
}

fn portal_can_encapsulate_origin(portal: &CCDSimplex,
                                 v4: &CCDSupport,
                                 dir: &Vector3f) -> bool {
    let dot = v4.v.dot(&dir);
    return is_zero_approx(dot) || dot > CCD_ZERO;
}

fn portal_reach_tolerance(portal: &CCDSimplex,
                          v4: &CCDSupport,
                          dir: &Vector3f,
                          ccd: &CCDCriteria) -> bool {
    let dv1 = portal.point(1).v.dot(&dir);
    let dv2 = portal.point(2).v.dot(&dir);
    let dv3 = portal.point(3).v.dot(&dir);
    let dv4 = v4.v.dot(&dir);

    let dot1 = dv4 - dv1;
    let dot2 = dv4 - dv2;
    let dot3 = dv4 - dv3;

    let dot1 = dot1.min(dot2);
    let dot1 = dot1.min(dot3);

    return scalar_eq_approx(dot1, ccd.mpr_tolerance) || dot1 < ccd.mpr_tolerance;
}

fn expand_portal(portal: &mut CCDSimplex, v4: &CCDSupport) {
    let v4v0 = v4.v.cross(&portal.point(0).v);
    let dot  = portal.point(1).v.dot(&v4v0);
    if dot > CCD_ZERO {
        let dot = portal.point(2).v.dot(&v4v0);
        if dot > CCD_ZERO {
            portal.set(1, v4.clone());
        } else {
            portal.set(3, v4.clone());
        }
    } else {
        let dot = portal.point(3).v.dot(&v4v0);
        if dot > CCD_ZERO {
            portal.set(2, v4.clone());
        } else {
            portal.set(1, v4.clone());
        }
    }
}