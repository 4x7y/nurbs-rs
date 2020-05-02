use crate::math::{MatrixDDf, VectorDf};


fn solve_l1_2(
    L: &MatrixDDf,
    B: &MatrixDDf,
    n: usize,
    l_skip1: usize,
) {
    /* compute all 2 x 2 blocks of X */
    for i in (0..n).step_by(2) {
        /* compute all 2 x 2 block of X, from rows i..i+2-1 */
        /* set the Z matrix to 0 */
        Z11=0;
        Z12=0;
        Z21=0;
        Z22=0;
        ell = L + i*lskip1;
        ex = B;
        /* the inner loop that computes outer products and adds them to Z */
        for j in (0..=i-2).rev().step_by(-2) {
            /* compute outer product and add it to the Z matrix */
            p1=ell[0];
            q1=ex[0];
            m11 = p1 * q1;
            q2=ex[lskip1];
            m12 = p1 * q2;
            p2=ell[lskip1];
            m21 = p2 * q1;
            m22 = p2 * q2;
            Z11 += m11;
            Z12 += m12;
            Z21 += m21;
            Z22 += m22;
            /* compute outer product and add it to the Z matrix */
            p1=ell[1];
            q1=ex[1];
            m11 = p1 * q1;
            q2=ex[1+lskip1];
            m12 = p1 * q2;
            p2=ell[1+lskip1];
            m21 = p2 * q1;
            m22 = p2 * q2;
            /* advance pointers */
            ell += 2;
            ex += 2;
            Z11 += m11;
            Z12 += m12;
            Z21 += m21;
            Z22 += m22;
            /* end of inner loop */
        }
        /* compute left-over iterations */
        j += 2;
        for (; j > 0; j--) {
            /* compute outer product and add it to the Z matrix */
            p1=ell[0];
            q1=ex[0];
            m11 = p1 * q1;
            q2=ex[lskip1];
            m12 = p1 * q2;
            p2=ell[lskip1];
            m21 = p2 * q1;
            m22 = p2 * q2;
            /* advance pointers */
            ell += 1;
            ex += 1;
            Z11 += m11;
            Z12 += m12;
            Z21 += m21;
            Z22 += m22;
        }
        /* finish computing the X(i) block */
        Z11 = ex[0] - Z11;
        ex[0] = Z11;
        Z12 = ex[lskip1] - Z12;
        ex[lskip1] = Z12;
        p1 = ell[lskip1];
        Z21 = ex[1] - Z21 - p1*Z11;
        ex[1] = Z21;
        Z22 = ex[1+lskip1] - Z22 - p1*Z12;
        ex[1+lskip1] = Z22;
        /* end of outer loop */
    }
}


pub fn factor_ldlt(
    A: &MatrixDDf,
    d: &VectorDf,
    n: usize,
    n_skip1: usize,
) {

}