#include <cassert>
#include <type_traits>
#include <Eigen/QR>
#include "exception.h"
/*
 * Member function definitions of pole namespace template functions.
 * See pole.h for template class declaration.
 */

namespace pole {
namespace internal {

template <typename DerivedA, typename DerivedB>
Eigen::Matrix<real_t, DerivedB::ColsAtCompileTime, DerivedB::RowsAtCompileTime>
    place_full_rank(const Eigen::MatrixBase<DerivedA>& A,
                    const DerivedB& qr_B,
                    const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedA::RowsAtCompileTime, 1>>& sorted_poles) {
    static_assert(DerivedA::RowsAtCompileTime == DerivedA::ColsAtCompileTime, "A must be square");
    static_assert(DerivedA::RowsAtCompileTime == DerivedB::RowsAtCompileTime, "Row size of A and B must be equal");

    static constexpr auto n = DerivedA::RowsAtCompileTime;

    Eigen::Matrix<real_t, n, n> diagonal_pole_matrix = Eigen::Matrix<real_t, n, n>::Zero();
    for (unsigned int j = 0; j < n; ++j) {
        pole_t p = sorted_poles(j);
        diagonal_pole_matrix(j, j) = p.real();
        /*
         * If p is complex, we use a _real_ block matrix with eigenvalues identical to the conjugate pair
         * [a+bi    0] => [a -b]
         * [   0 a-bi]    [b  a]
         */
        if (!is_real(p)) {
            diagonal_pole_matrix(j, j + 1) = -p.imag();
            diagonal_pole_matrix(j + 1, j) = p.imag();
            diagonal_pole_matrix(j + 1, j + 1) = p.real();

            /* assert the next pole is the complex conjugate */
            assert(p == std::conj(sorted_poles(j + 1)));
            ++j;
        }
    }

    /*
     * Use least squares to solve:
     * (A + BK) = diag(P) => BK = (A - diag(P))
     */
    return qr_B.solve(diagonal_pole_matrix - A);
}

template <typename DerivedS, typename DerivedX, typename DerivedB>
void yt_loop(Eigen::MatrixBase<DerivedS>& S,
             Eigen::MatrixBase<DerivedX>& X,
             const Eigen::MatrixBase<DerivedB>& B,
             const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedB::RowsAtCompileTime, 1>>& poles) {
    static_assert(DerivedS::RowsAtCompileTime == DerivedX::ColsAtCompileTime, "Row size of S and X must be square");
    static_assert(DerivedS::RowsAtCompileTime == DerivedB::RowsAtCompileTime, "Row size of S and B must be equal");
    static_assert(DerivedB::RowsAtCompileTime*DerivedB::ColsAtCompileTime == DerivedS::ColsAtCompileTime,
            "Col size of S must be equal to (row size B)*(col size B)");

    throw NotImplementedException();
}

} // namespace internal

template <typename T>
T continuous_to_discrete(const Eigen::DenseBase<T>& poles, real_t dt) {
    static_assert(std::is_same<typename T::Scalar, pole_t>::value,
            "Invalid scalar type for converting poles from continuous to discrete time");
    static_assert(T::ColsAtCompileTime == 1, "Input must be a vector of poles");

    T discrete_poles;
    for (unsigned int j = 0; j < poles.size(); ++j) {
        discrete_poles(j) = continuous_to_discrete(poles(j), dt);
    }
    return discrete_poles;
}

template <typename DerivedA, typename DerivedB>
Eigen::Matrix<real_t, DerivedB::ColsAtCompileTime, DerivedB::RowsAtCompileTime>
    place(const Eigen::MatrixBase<DerivedA>& A,
          const Eigen::MatrixBase<DerivedB>& B,
          const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedA::RowsAtCompileTime, 1>>& poles) {
    static_assert(DerivedA::RowsAtCompileTime == DerivedA::ColsAtCompileTime, "A must be square");
    static_assert(DerivedA::RowsAtCompileTime == DerivedB::RowsAtCompileTime, "Row size of A and B must be equal");
    /* NOTE: KNV solves A + BF but the usual form is A - BF so the returned value is negated. */

    using complex_t = pole_t;
    using namespace Eigen;
    static constexpr auto n = DerivedB::RowsAtCompileTime;
    static constexpr auto m = DerivedB::ColsAtCompileTime;
    Eigen::Matrix<pole_t, n, 1> sorted_poles = poles;
    std::sort(sorted_poles.data(), sorted_poles.data() + n, pole::internal::compare);

    if (m >= n) {
        const ColPivHouseholderQR<typename MatrixBase<DerivedB>::PlainObject> col_qr_B = B.colPivHouseholderQr();
        assert(col_qr_B.rank() == n); /* B is full rank */
        return -1*internal::place_full_rank(A, col_qr_B, sorted_poles);
    }
    assert(B.colPivHouseholderQr().rank() == m); /* B is full rank */

    /*
     * step A: QR decomposition of B, page 1132 KNV
     *
     * We use the QR decomposition with column pivoting only to determine rank and to solve the
     * fully/over-constrained system. Now we recompute the QR decomposition without pivoting to
     * determine as is done in the KNV paper.
     */
    const HouseholderQR<typename MatrixBase<DerivedB>::PlainObject> qr_B = B.householderQr();
    const Matrix<real_t, n, n> Q = qr_B.householderQ();
    const Matrix<real_t, n, m> U0 = Q.template leftCols<m>();
    const Matrix<real_t, n, n - m> U1 = Q.template rightCols<n - m>();
    const Matrix<real_t, m, m> Z = qr_B.matrixQR().template topRows<m>();

    static const Matrix<real_t, n, n> I = Matrix<real_t, n, n>::Identity();
    Matrix<real_t, n, n> X; /* transfer matrix */
    Matrix<complex_t, n, n*m> S; /* ker_pole */

    for (unsigned int j = 0; j < n; ++j) {
        const complex_t p = sorted_poles(j);
        const Matrix<complex_t, n, n - m> Pj = (U1.transpose()*(A - p*I)).transpose();
        const Matrix<complex_t, n, n> Qj = Pj.householderQr().householderQ();
        const Matrix<complex_t, n, m> Sj = Qj.template rightCols<m>();
        const Matrix<complex_t, n, 1> Xj = Sj.rowwise().sum().normalized();

        if (!is_real(p)) {
            S.template block<n, m>(0, j*m) = Sj;
            S.template block<n, m>(0, (j + 1)*m) = Sj;
            X.col(j) = Xj.real();
            X.col(j) = Xj.imag();

            /* assert the next pole is the complex conjugate */
            assert(p == std::conj(sorted_poles(j + 1)));
            ++j;
        } else {
            S.template block<n, m>(0, j*m) = Sj;
            X.col(j) = Xj.real();
        }
    }

    internal::yt_loop(S, X, B, sorted_poles);

    /*
     * step F: calculation of feedback gain F
     *
     * Need to reconstruct X for to match complex conjugate pairs
     */
    Matrix<complex_t, n, n> Xc = X;
    for (unsigned int j = 0; j < n; ++j) {
        const complex_t p = sorted_poles(j);
        if(!is_real(p)) {
            static constexpr complex_t i = complex_t{static_cast<real_t>(0), static_cast<real_t>(1)};

            const Matrix<complex_t, n, 1> real_part = Xc.col(j);
            const Matrix<complex_t, n, 1> imag_part = Xc.col(j + 1);
            Xc.col(j) = real_part - i*imag_part;
            Xc.col(j + 1) = real_part + i*imag_part;
        }
    }

    /* M*X = X*diag(P) */
    Matrix<complex_t, n, n> M = Xc.transpose().colPivHouseholderQr().solve(sorted_poles.asDiagonal()*Xc.transpose());
    /* ZF = U0'*(M - A) */
    return -1*Z.template triangularView<Upper>().solve(U0.transpose()*(M.real() - A));
}

} // namespace pole
