#pragma once
#include <type_traits>
#include <Eigen/Core>
#include <Eigen/QR>
#include "discrete_linear.h"

namespace controller {
// TODO: Allow reference state to vary in horizon
using real_t = model::real_t;

template<typename Model>
class Lqr {
    static_assert(std::is_base_of<model::DiscreteLinearBase, Model>::value, "Invalid template parameter type for Lqr");
    public:
        using state_t = typename Model::state_t;
        using input_t = typename Model::input_t;
        using state_matrix_t = typename Model::state_matrix_t;
        using input_matrix_t = typename Model::input_matrix_t;
        using lqr_gain_t = typename Eigen::Matrix<real_t, Model::m, Model::n>;
        using state_cost_t = state_matrix_t;
        using input_cost_t = typename Eigen::Matrix<real_t, Model::m, Model::m>;

        Lqr(Model& system, const state_cost_t& Q, const input_cost_t& R,
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
        Model& system() const;
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
        using control_mask_t = typename Eigen::Matrix<uint32_t, Model::m, 1>;
        using augmented_state_matrix_t = typename Eigen::Matrix<real_t, 2*Model::n, 2*Model::n>;
        using augmented_input_matrix_t = typename Eigen::Matrix<real_t, 2*Model::n, Model::m>;
        using augmented_lqr_gain_t = typename Eigen::Matrix<real_t, Model::m, 2*Model::n>;
        using augmented_state_cost_t = augmented_state_matrix_t;

        Model& m_system;                        // controlled system or plant
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

template<typename Model>
inline void Lqr<Model>::set_horizon(uint32_t horizon_iterations) {
    m_horizon = horizon_iterations;
}

template<typename Model>
inline void Lqr<Model>::set_reference(const state_t& r) {
    m_r = r;
}

template<typename Model>
inline void Lqr<Model>::set_error_integral(const state_t& q) {
    m_q = q;
}

template<typename Model>
inline void Lqr<Model>::set_Q(const state_cost_t& Q) {
    m_steady_state = false;
    m_Q = Q;
}

template<typename Model>
inline void Lqr<Model>::set_Qi(const state_cost_t& Qi) {
    m_steady_state = false;
    m_Qi = Qi;
    // Look at diagonal entries of Qi and if zero, treat the output as unobserved.
    m_Ag.template bottomLeftCorner<Model::n, Model::n>() =
        (m_Qi.diagonal().array() != 0.0).template cast<real_t>().matrix().asDiagonal();
}

template<typename Model>
inline void Lqr<Model>::set_R(const input_cost_t& R) {
    m_steady_state = false;
    m_R = R;
    set_control_mask();
}

template<typename Model>
inline Model& Lqr<Model>::system() const {
    return m_system;
}

template<typename Model>
inline uint32_t Lqr<Model>::horizon_iterations() const {
    return m_horizon;
}

template<typename Model>
inline const typename Lqr<Model>::state_t& Lqr<Model>::r() const {
    return m_r;
}

template<typename Model>
inline const typename Lqr<Model>::state_t& Lqr<Model>::q() const {
    return m_q;
}

template<typename Model>
inline typename Lqr<Model>::lqr_gain_t Lqr<Model>::K() const {
    return m_Kg.template leftCols<Model::n>();
}

template<typename Model>
inline typename Lqr<Model>::lqr_gain_t Lqr<Model>::Ki() const {
    return m_Kg.template rightCols<Model::n>();
}

template<typename Model>
inline typename Lqr<Model>::state_cost_t Lqr<Model>::P() const {
    return m_Pg.template topLeftCorner<Model::n, Model::n>();
}

template<typename Model>
inline const typename Lqr<Model>::state_cost_t& Lqr<Model>::Q() const {
    return m_Q;
}

template<typename Model>
inline const typename Lqr<Model>::state_cost_t& Lqr<Model>::Qi() const {
    return m_Qi;
}

template<typename Model>
inline const typename Lqr<Model>::input_cost_t& Lqr<Model>::R() const {
    return m_R;
}

template<typename Model>
inline real_t Lqr<Model>::dt() const {
    return m_system.dt();
}

} // namespace controller

#include "lqr.hh"
