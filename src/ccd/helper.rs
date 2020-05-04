use crate::math::{Vector3f, Scalar};
use crate::ccd::{CCDCriteria, CCDObject};

pub(crate) const CCD_EPS: Scalar = 1e-6;
pub(crate) const CCD_ZERO: Scalar = 0.;
pub(crate) const CCD_ONE: Scalar = 1.;

// pub(crate) const CCD_VEC3F_ORIGIN: Vector3f = Vector3f::zeros();

pub(crate) fn is_zero_approx(a: Scalar) -> bool {
    a.abs() < CCD_EPS
}

pub(crate) fn vec_eq_approx(a: &Vector3f, b: &Vector3f) -> bool {
    for i in 0..3 {
        let ab = (a[i] - b[i]).abs();
        if ab < CCD_EPS {
            continue;
        }

        let abs_a = a[i].abs();
        let abs_b = b[i].abs();
        if abs_b > abs_a {
            if ab > CCD_EPS * abs_b {
                return false;
            }
        } else {
            if ab > CCD_EPS * abs_b {
                return false;
            }
        }
    }

    return true;
}


pub(crate) fn scalar_eq_approx(a: Scalar, b: Scalar) -> bool {
    let ab = (a - b).abs();
    if ab < CCD_EPS {
        return true;
    }

    let abs_a = a.abs();
    let abs_b = b.abs();
    if abs_b > abs_a {
        if ab < CCD_EPS * abs_b {
            return true;
        }
    } else {
        if ab < CCD_EPS * abs_b {
            return true;
        }
    }

    return false;
}

/// Returns distance^2 of point P from triangle formed by triplet a, b, c.
/// If witness vector is provided it is filled with coordinates of point
/// from which was computed distance to point P.
pub(crate) fn vec_point_triangle_dist_sq(P: &Vector3f,
                                         x0: &Vector3f,
                                         B: &Vector3f,
                                         C: &Vector3f,
                                         witness: &mut Vector3f) -> Scalar {
    // Computation comes from analytic expression for triangle (x0, B, C)
    //      T(s, t) = x0 + s.d1 + t.d2, where d1 = B - x0 and d2 = C - x0 and
    // Then equation for distance is:
    //      D(s, t) = | T(s, t) - P |^2
    // This leads to minimization of quadratic function of two variables.
    // The solution from is taken only if s is between 0 and 1, t is
    // between 0 and 1 and t + s < 1, otherwise distance from segment is
    // computed.

    let mut d1: Vector3f = B - x0;
    let mut d2: Vector3f = C - x0;
    let  a: Vector3f = x0 - P;

    let u = a.dot(&a);
    let v = d1.dot(&d1);
    let w = d2.dot(&d2);
    let p = a.dot(&d1);
    let q = a.dot(&d2);
    let r = d1.dot(&d2);

    let d = w * v - r * r;
    let s: Scalar;
    let t: Scalar;
    if is_zero_approx(d) {
        s = -1.;
        t = -1.;
    } else {
        s = (q * r - w * p) / d;
        t = (-s * r - q) / w;
    }

    let mut dist;
    if (is_zero_approx(s) || s > CCD_ZERO)
        && (scalar_eq_approx(s, CCD_ONE) || s < CCD_ONE)
        && (is_zero_approx(t) || t > CCD_ZERO)
        && (scalar_eq_approx(t, CCD_ONE) || t < CCD_ONE)
        && (scalar_eq_approx(t + s, CCD_ONE) || t + s < CCD_ONE) {

        // if witness vector is provided
        d1.scale_mut(s);
        d2.scale_mut(t);
        witness.copy_from(&(x0 + &d1 + &d2));
        dist = (*witness - P).norm_squared();
    } else {
        dist = vec_point_segment_dist_sq(P, x0, B, witness);
        let mut witness2 = Vector3f::zeros();
        let dist2 = vec_point_segment_dist_sq(P, x0, C, &mut witness2);
        if dist2 < dist {
            dist = dist2;
            witness.copy_from(&witness2);
        }

        let dist2 = vec_point_segment_dist_sq(P, B, C, &mut witness2);
        if dist2 < dist {
            dist = dist2;
            witness.copy_from(&witness2);
        }
    }

    return dist;
}

fn vec_point_segment_dist_sq(P: &Vector3f,
                             x0: &Vector3f,
                             b: &Vector3f,
                             witness: &mut Vector3f) -> Scalar {

    // The computation comes from solving equation of segment:
    //      S(t) = x0 + t.d
    //          where - x0 is initial point of segment
    //                - d is direction of segment from x0 (|d| > 0)
    //                - t belongs to <0, 1> interval
    //
    // Than, distance from a segment to some point P can be expressed:
    //      D(t) = |x0 + t.d - P|^2
    //          which is distance from any point on segment. Minimization
    //          of this function brings distance from P to segment.
    // Minimization of D(t) leads to simple quadratic equation that's
    // solving is straightforward.
    //
    // Bonus of this method is witness point for free.

    // direction of segment
    let d: Vector3f = b - x0;

    // pre-compute vector from P to x0
    let a: Vector3f = x0 - P;

    let mut t  = -CCD_ONE * a.dot(&d);
    t /= d.norm_squared();

    let dist;
    if t < CCD_ZERO || is_zero_approx(t) {
        dist = (x0 - P).norm_squared();
        witness.copy_from(x0);
    } else if t > CCD_ONE || scalar_eq_approx(t, CCD_ONE) {
        dist = (b - P).norm_squared();
        witness.copy_from(b);
    } else {
        witness.copy_from(&(d * t + x0));
        dist = (*witness - P).norm_squared();
    }

    return dist;
}

#[derive(Debug, Clone)]
pub struct CCDSupport {
    pub v:  Vector3f,             // Support point in minkowski sum
    pub v1: Vector3f,             // Support point in obj1
    pub v2: Vector3f,             // Support point in obj2
}


#[derive(Debug, Clone)]
pub struct CCDSimplex {
    ps:   Vec<CCDSupport>,
    last: i32,
}


impl CCDSupport {

    pub fn new() -> Self {
        CCDSupport {
            v:  Vector3f::zeros(),
            v1: Vector3f::zeros(),
            v2: Vector3f::zeros(),
        }
    }

    pub fn from(obj1: &dyn CCDObject,
                obj2: &dyn CCDObject,
                dir: &Vector3f,
                ccd: &CCDCriteria) -> Self {
        let v1 = obj1.support(dir);
        let v2 = obj2.support(&(-1. * dir));
        let v  = v1 - &v2;
        CCDSupport {
            v1: v1,
            v2: v2,
            v:  v,
        }
    }

    pub fn initialize(&mut self,
                      obj1: &dyn CCDObject,
                      obj2: &dyn CCDObject,
                      dir: &Vector3f,
                      ccd: &CCDCriteria) {
        self.v1 = obj1.support(dir);
        self.v2 = obj2.support(&(-1. * dir));
        self.v  = self.v1 - &self.v2;
    }
}


impl CCDSimplex {

    pub fn new() -> Self {
        CCDSimplex {
            ps: vec![CCDSupport::new(); 4],
            last: -1,
        }
    }

    pub fn size(&self) -> usize {
        assert!(self.last >= -1 && self.last <= 3);
        (self.last + 1) as usize
    }

    pub fn last(&self) -> &CCDSupport {
        assert!(self.last >= 0);
        self.ps.get(self.last as usize).unwrap()
    }

    pub fn point(&self, index: usize) -> &CCDSupport {
        // assert!(index < 4 && index as i32 <= self.last);
        self.ps.get(index).unwrap()
    }

    pub fn point_mut(&mut self, index: usize) -> &mut CCDSupport {
        // assert!(index < 4 && index as i32 <= self.last);
        self.ps.get_mut(index).unwrap()
    }

    pub fn add(&mut self, v: CCDSupport) {
        assert!(self.last < 3);
        self.last += 1;
        self.ps[self.last as usize] = v;
    }

    pub fn set(&mut self, pos: usize, v: CCDSupport) {
        assert!(pos < 4 && pos as i32 <= self.last);
        self.ps[pos] = v;
    }

    pub fn set_size(&mut self, size: usize) {
        assert!(size <= 4);
        self.last = size as i32 - 1;
    }

    pub fn swap(&mut self, pos1: usize, pos2: usize) {
        assert!(pos1 < 4 && pos1 as i32 <= self.last);
        assert!(pos2 < 4 && pos2 as i32 <= self.last);
        self.ps.swap(pos1, pos2);
    }

    pub fn cross_self_point_v(&self, pos1: usize, pos2: usize) -> Vector3f {
        self.ps[pos1].v.cross(&self.ps[pos2].v)
    }

    pub fn dot_point_v_with(&self, pos: usize, other: &Vector3f) -> Scalar {
        self.ps[pos].v.dot(other)
    }
}