
/// Return zero 2D matrix with dimension row x col
///
/// # Arguments
///
/// * `row` - number of rows
/// * `col` - number of cols
pub fn zeros(row: usize, col: usize) -> Vec<Vec<f32>> {
    vec![vec![0f32; col]; row]
}

/// Return zero 3D tensor with dimension d1xd2xd3
pub fn zeros_3d(d1: usize, d2: usize, d3: usize) -> Vec<Vec<Vec<f32>>> {
    vec![vec![vec![0f32; d3]; d2]; d1]
}

/// Caps a value inside a certain range.
#[inline]
pub fn limit_range<T>(min: T, max: T, value: T) -> T
    where T: PartialOrd {
    if value > max {
        max
    }
    else if value < min {
        min
    } else {
        value
    }
}

pub(crate) const ERROR_CODE_URDF_PARSING: i32 = 1;