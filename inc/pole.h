#pragma once
#include <cmath>
#include <complex>
#include <Eigen/Core>

namespace pole {

using real_t = model::real_t;
using pole_t = std::complex<real_t>;

bool is_real(const pole_t& p) { return p.imag() == static_cast<real_t>(0); }

pole_t continuous_to_discrete(const pole_t& p, real_t dt) { return std::exp(p*dt); }

template <typename T>
T continuous_to_discrete(const Eigen::DenseBase<T>& poles, real_t dt);
namespace internal {

template <typename DerivedA, typename DerivedB>
Eigen::Matrix<real_t, DerivedB::ColsAtCompileTime, DerivedB::RowsAtCompileTime>
    place_full_rank(const Eigen::MatrixBase<DerivedA>& A,
                    const DerivedB& qr_B,
                    const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedA::RowsAtCompileTime, 1>>& poles);

template <typename DerivedS, typename DerivedX, typename DerivedB>
void yt_loop(Eigen::MatrixBase<DerivedS>& S,
             Eigen::MatrixBase<DerivedX>& X,
             const Eigen::MatrixBase<DerivedB>& B,
             const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedB::RowsAtCompileTime, 1>>& poles);

bool compare(const pole_t& lhs, const pole_t& rhs) {
    /*
     * Pole comparison function to be used with sort. This orders poles such that
     * real poles come first and complex poles are placed to their conjugate pair
     * as is described on page 13 YT.
     */
    if (is_real(lhs)) {
        if (is_real(rhs)) {
            return lhs.real() < rhs.real();
        } else { /* lhs is real and rhs is complex */
            return true;
        }
    } else {
        if (is_real(rhs)) { /* lhs is complex and rhs is real */
            return false;
        } else { /* both are complex */
            /* We want complex conjugate pairs to be placed adjacently. */
            if (lhs.real() != rhs.real()) {
                return lhs.real() < rhs.real();
            } else { /* real parts are the same */
                /*
                 * If the magnitude of imaginary parts are different, return
                 * a comparison of the magnitudes. Smaller magnitudes come first.
                 */
                if (std::abs(lhs.imag()) != std::abs(rhs.imag())) {
                    return std::abs(lhs.imag()) < std::abs(rhs.imag());
                } else {
                    return lhs.imag() < rhs.imag();
                }
            }
        }
    }
}

} // namespace internal


template <typename T>
T sort(const Eigen::DenseBase<T>& poles);

template <typename DerivedA, typename DerivedB>
Eigen::Matrix<real_t, DerivedB::ColsAtCompileTime, DerivedB::RowsAtCompileTime>
    place(const Eigen::MatrixBase<DerivedA>& A,
          const Eigen::MatrixBase<DerivedB>& B,
          const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedA::RowsAtCompileTime, 1>>& poles);

} // namespace pole

#include "pole.hh"
