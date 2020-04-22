use crate::math::*;

struct LCP {
    n: usize,
    dell: VectorDf,
    ell: VectorDf,
}

impl LCP {

    pub fn new() {

    }

    pub fn transfer_i_to_c(&self) {

    }

    pub fn solve(&self) {

    }

    pub fn unpermute(&self) {

    }
}



/// Solve `A*x = b+w`, with `x` and `w` subject to certain LCP conditions.
/// Each `x(i)`, `w(i)` must lie on one of the three line segments in the
/// following diagram. Each line segment corresponds to one index set :
///
///      w(i)
///      /|\      |           :
///       |       |           :
///       |       |i in N     :
///   w>0 |       |state[i]=0 :
///       |       |           :
///       |       |           :  i in C
///   w=0 +       +-----------------------+
///       |                   :           |
///       |                   :           |
///   w<0 |                   :           |i in N
///       |                   :           |state[i]=1
///       |                   :           |
///       |                   :           |
///       +-------|-----------|-----------|----------> x(i)
///              lo           0           hi
///
/// The Dantzig algorithm proceeds as follows:
///   for i=1:n
///     * if (x(i),w(i)) is not on the line, push x(i) and w(i) positive or
///       negative towards the line. as this is done, the other (x(j),w(j))
///       for j<i are constrained to be on the line. if any (x,w) reaches the
///       end of a line segment then it is switched between index sets.
///     * i is added to the appropriate index set depending on what line segment
///       it hits.
/// we restrict lo(i) <= 0 and hi(i) >= 0. this makes the algorithm a bit
/// simpler, because the starting point for x(i),w(i) is always on the dotted
/// line x=0 and x will only ever increase in one direction, so it can only hit
/// two out of the three line segments.
///
/// # NOTES
///
/// This is an implementation of "lcp_dantzig2_ldlt.m" and "lcp_dantzig_lohi.m".
/// the implementation is split into an LCP problem object (dLCP) and an LCP
/// driver function. most optimization occurs in the dLCP object.
/// a naive implementation of the algorithm requires either a lot of data motion
/// or a lot of permutation-array lookup, because we are constantly re-ordering
/// rows and columns. to avoid this and make a more optimized algorithm, a
/// non-trivial data structure is used to represent the matrix A (this is
/// implemented in the fast version of the dLCP object).
/// during execution of this algorithm, some indexes in A are clamped (set `C`),
/// some are non-clamped (set N), and some are "don't care" (where `x=0`).
/// A,x,b,w (and other problem vectors) are permuted such that the clamped
/// indexes are first, the unclamped indexes are next, and the don't-care
/// indexes are last. this permutation is recorded in the array `p`.
/// initially `p = 0..n-1`, and as the rows and columns of `A,x,b,w` are swapped,
/// the corresponding elements of p are swapped.
///
/// Because the C and N elements are grouped together in the rows of A, we can do
/// lots of work with a fast dot product function. if A,x,etc were not permuted
/// and we only had a permutation array, then those dot products would be much
/// slower as we would have a permutation array lookup in some inner loops.
///
/// `A` is accessed through an array of row pointers, so that element (i,j) of the
/// permuted matrix is `A[i][j]`. this makes row swapping fast. for column swapping
/// we still have to actually move the data.
///
/// During execution of this algorithm we maintain an `L*D*L'` factorization of
/// The clamped sub-matrix of `A` (call it `AC`) which is the top left `nC*nC`
/// sub-matrix of A. there are two ways we could arrange the rows/columns in AC.
///
/// 1. `AC` is always permuted such that `L*D*L' = AC`. this causes a problem
/// when a row/column is removed from `C`, because then all the rows/columns of A
/// between the deleted index and the end of C need to be rotated downward.
/// this results in a lot of data motion and slows things down.
///
/// 2. `L*D*L'` is actually a factorization of a *permutation* of `AC` (which is
/// itself a permutation of the underlying `A`). this is what we do - the
/// permutation is recorded in the vector `C`. call this permutation `A[C,C]`.
/// when a row/column is removed from `C`, all we have to do is swap two
/// rows/columns and manipulate `C`.

/// https://github.com/dartsim/dart/blob/master/dart/external/odelcpsolver/lcp.cpp
/// Swap both row and column `i1` with `i2` in the lower triangle of `n*n`
/// matrix `A`.
fn swap_rows_and_cols(
    A: MatrixDDf,
    i1: usize,
    i2: usize
) {

}


/// Swap two

pub fn solve(A: MatrixDDf,
             b: VectorDf,
             x: VectorDf,
             num_contacts: usize, mu: Scalar, num_dir: usize) {


}