#pragma once
#include <type_traits>
#include <Eigen/Core>
#include <Eigen/QR>
#include "discrete_linear.h"

namespace controller {
// TODO: Allow reference state to vary in horizon
using real_t = model::real_t;

template<typename T>
class Lqr {
    static_assert(
            (std::is_base_of<model::LinearBase, T>::value || std::is_base_of<model::DiscreteLinearBase, T>::value),
            "Invalid template parameter type for Lqr");
    public:
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using state_matrix_t = typename T::state_matrix_t;
        using input_matrix_t = typename T::input_matrix_t;
        using lqr_gain_t = typename Eigen::Matrix<real_t, T::m, T::n>;
        using state_cost_t = state_matrix_t;
        using input_cost_t = typename Eigen::Matrix<real_t, T::m, T::m>;

        Lqr(T& system, const state_cost_t& Q, const input_cost_t& R,
                const state_t& r, uint32_t horizon_iterations,
                const state_cost_t& Qi = state_cost_t::Zero(),
                const state_t& q = state_t::Zero());

        input_t control_calculate(const state_t& x);
        input_t control_calculate(const state_t& x, const state_t& r);

        void set_horizon(uint32_t horizon_iterations);
        void set_reference(const state_t& r);
        void set_error_integral(const state_t& q);
        void set_Q(const state_cost_t& Q);
        void set_Qi(const state_cost_t& Qi);
        void set_R(const input_cost_t& R);

        // accessors
        T& system() const;
        uint32_t horizon_iterations() const;
        const state_t& r() const;
        const state_t& q() const;
        lqr_gain_t K() const;
        lqr_gain_t Ki() const;
        state_cost_t P() const; // TODO: Also return augmented cost-to-go matrix?
        const state_cost_t& Q() const;
        const state_cost_t& Qi() const;
        const input_cost_t& R() const;
        real_t dt() const;

    private:
        using control_mask_t = typename Eigen::Matrix<uint32_t, T::m, 1>;
        using augmented_state_matrix_t = typename Eigen::Matrix<real_t, 2*T::n, 2*T::n>;
        using augmented_input_matrix_t = typename Eigen::Matrix<real_t, 2*T::n, T::m>;
        using augmented_lqr_gain_t = typename Eigen::Matrix<real_t, T::m, 2*T::n>;
        using augmented_state_cost_t = augmented_state_matrix_t;

        T& m_system;                        // controlled system or plant
        uint32_t m_horizon;                 // horizon length in iterations
        state_t m_r;                        // reference state
        state_t m_q;                        // error integral
        //lqr_gain_t m_K;                     // computed feedback gain
        //state_cost_t m_P;                   // cost-to-go
        state_cost_t m_Q;                   // state cost weights
        state_cost_t m_Qi;                  // error integral cost weights
        input_cost_t m_R;                   // input cost weights
        bool m_steady_state;                // steady state indicator
        //state_matrix_t m_Ad;                // copy of system state matrix
        input_matrix_t m_Bd;                // copy of system input matrix
        control_mask_t m_mask;              // control input mask
        uint32_t m_m;                       // number of accessible control inputs
        input_cost_t m_Rr;                  // reduced form of input cost matrix
        augmented_input_matrix_t m_Bg;      // augmented and reduced form of input matrix
        augmented_state_matrix_t m_Ag;      // augmented state matrix for integral action
        augmented_lqr_gain_t m_Kg;          // computed feedback gain (standard + integral/tracking)
        augmented_state_cost_t m_Pg;        // augmented cost-to-go matrix

        void perform_value_iteration();
        void update_lqr_gain();
        void update_horizon_cost();
        void set_control_mask();
        void reduce_input_matrices();
        void update_error_integral(const state_t& x);
}; // class Lqr

template<typename T>
inline void Lqr<T>::set_horizon(uint32_t horizon_iterations) {
    m_horizon = horizon_iterations;
}

template<typename T>
inline void Lqr<T>::set_reference(const state_t& r) {
    m_r = r;
}

template<typename T>
inline void Lqr<T>::set_error_integral(const state_t& q) {
    m_q = q;
}

template<typename T>
inline void Lqr<T>::set_Q(const state_cost_t& Q) {
    m_steady_state = false;
    m_Q = Q;
}

template<typename T>
inline void Lqr<T>::set_Qi(const state_cost_t& Qi) {
    m_steady_state = false;
    m_Qi = Qi;
    // Look at diagonal entries of Qi and if zero, treat the output as unobserved.
    m_Ag.template bottomLeftCorner<T::n, T::n>() =
        (m_Qi.diagonal().array() != 0.0).template cast<real_t>().matrix().asDiagonal();
}

template<typename T>
inline void Lqr<T>::set_R(const input_cost_t& R) {
    m_steady_state = false;
    m_R = R;
    set_control_mask();
}

template<typename T>
inline T& Lqr<T>::system() const {
    return m_system;
}

template<typename T>
inline uint32_t Lqr<T>::horizon_iterations() const {
    return m_horizon;
}

template<typename T>
inline const typename Lqr<T>::state_t& Lqr<T>::r() const {
    return m_r;
}

template<typename T>
inline const typename Lqr<T>::state_t& Lqr<T>::q() const {
    return m_q;
}

template<typename T>
inline typename Lqr<T>::lqr_gain_t Lqr<T>::K() const {
    return m_Kg.template leftCols<T::n>();
}

template<typename T>
inline typename Lqr<T>::lqr_gain_t Lqr<T>::Ki() const {
    return m_Kg.template rightCols<T::n>();
}

template<typename T>
inline typename Lqr<T>::state_cost_t Lqr<T>::P() const {
    return m_Pg.template topLeftCorner<T::n, T::n>();
}

template<typename T>
inline const typename Lqr<T>::state_cost_t& Lqr<T>::Q() const {
    return m_Q;
}

template<typename T>
inline const typename Lqr<T>::state_cost_t& Lqr<T>::Qi() const {
    return m_Qi;
}

template<typename T>
inline const typename Lqr<T>::input_cost_t& Lqr<T>::R() const {
    return m_R;
}

template<typename T>
inline real_t Lqr<T>::dt() const {
    return m_system.dt();
}

} // namespace controller

#include "lqr.hh"
