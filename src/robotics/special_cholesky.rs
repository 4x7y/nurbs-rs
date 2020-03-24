use crate::math::{MatrixDDf, VectorDf};

/// Sparsity-preserving Cholesky decomposition for rigid body
/// tree inertia matrix ONLY.
///
/// # Arguments
///
/// - `mmat`: inertia matrix of a rigid body tree               (nv x nv)
/// - `lambda`: index of the last non-zero of row i below the   (nv x 1)
///             main diagonal.
///
/// The output, L, is a lower triangular matrix
/// that preserves the sparsity of inertiaMatrix, and L'*L = inertiaMatrix.
pub fn special_cholesky(mmat: &MatrixDDf, lambda: &Vec<Option<usize>>) -> MatrixDDf {
    let nv = mmat.ncols();
    let mut mmat = mmat.clone_owned();

    for i in (0..nv).rev() {
        mmat[(i, i)] = mmat[(i, i)].sqrt();
        let mut k_opt = lambda[i];
        while let Some(k) = k_opt {
            mmat[(i, k)] = mmat[(i, k)] / mmat[(i, i)];
            k_opt = lambda[k];
        }

        k_opt = lambda[i];
        while let Some(k) = k_opt {
            let mut j_opt = Some(k);
            while let Some(j) = j_opt {
                mmat[(k, j)] = mmat[(k, j)] - mmat[(i, k)] * mmat[(i, j)];
                j_opt = lambda[j];
            }
            k_opt = lambda[k];
        }
    }

    return mmat.lower_triangle();
}

