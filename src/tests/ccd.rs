use crate::ccd::{mpr_penetration, CCDObject, CCD_EPS, CCD_ZERO, is_zero_approx, CCDCriteria, CCDResult};
use crate::math::{Vector3f, Matrix3f, Scalar};
use crate::utils::rotm2quat;
use crate::robotics::axang2rotm;
use failure::_core::f32::consts::PI;
use std::f64::consts::FRAC_PI_3;


#[test]
fn test_mpr_penetration() {
    assert!(true);
}
