use crate::math::{MatrixDDf, VectorDf};

/// Sparsity-preserving Cholesky decomposition for rigid body
/// tree inertia matrix ONLY.
///
/// # Arguments
/// The first input argument, inertiaMatrix, is
/// a valid vNum x vNum inertia matrix of a rigid body tree, where vNum is
/// the number of velocity variables. The second input, lambda, is a vNum x 1
/// vector. lambda(i) corresponds to the index of the last non-zero of row
/// i below the main diagonal. The output, L, is a lower trangular matrix
/// that preserves the sparsity of inertiaMatrix, and L'*L = inertiaMatrix.
pub fn special_cholesky(inertia: &MatrixDDf, lambda: &Vec<usize>) -> MatrixDDf {
    let n = inertia.ncols();
    let mut H = inertia.clone_owned();

    for i in (0..n).rev() {
        H[(i, i)] = H[(i, i)].sqrt();
        let mut k = lambda[i];
        while k > 0 {
            H[(i, k)] = H[(i, k)] / H[(i, i)];
            k = lambda[k];
        }

        k = lambda[i];
        while k > 0 {
            let mut j = k;
            while j > 0 {
                H[(k, j)] = H[(k, j)] - H[(i, k)] * H[(i, j)];
                j = lambda[j];
            }
            k = lambda[k];
        }
    }

    return H.lower_triangle();
}

