use crate::math::matrix::{Matrix3f, Vector3f};
use std::fmt::Display;
use failure::_core::fmt::{Formatter, Error};
use std::fmt;

pub enum EulerAngle {
    ZYX(Vector3f),
    XYZ(Vector3f),
    ZYZ(Vector3f),
}

impl Display for EulerAngle {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}", match self {
            EulerAngle::ZYX( eulr ) => {
                format!("{} {} {} (ZYX)", eulr[0], eulr[1], eulr[2])
            },
            EulerAngle::XYZ( eulr ) => {
                format!("{} {} {} (XYZ)", eulr[0], eulr[1], eulr[2])
            },
            EulerAngle::ZYZ( eulr ) => {
                format!("{} {} {} (ZYZ)", eulr[0], eulr[1], eulr[2])
            },
        })
    }
}

pub enum EulerAngleOrder {
    ZYX,
    XYZ,
    ZYZ,
}