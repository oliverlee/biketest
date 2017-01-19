/*
 * Member function definitions of Lqr template class.
 * See lqr.h for template class declaration.
 */

// FIXME: Allow use with DiscreateLinear OR Linear classes

namespace controller {

template<typename T>
Lqr<T>::Lqr(T& system, const state_cost_t& Q, const input_cost_t& R,
        const state_t& r, uint32_t horizon_iterations,
        const state_cost_t& Qi, const state_t& q) :
    m_system(system), m_horizon(horizon_iterations), m_r(r), m_q(q),
    m_Q(Q), m_Qi(Qi), m_R(R), m_steady_state(false),
    m_Bd(system.Bd()), m_Ag(augmented_state_matrix_t::Identity()),
    m_Pg(augmented_state_cost_t::Zero()) {
    m_Ag.template topLeftCorner<T::n, T::n>() = m_system.Ad();
    m_Ag.template bottomLeftCorner<T::n, T::n>() =
        (m_Qi.diagonal().array() != 0.0).template cast<real_t>().matrix().asDiagonal();
    set_control_mask();
}

template<typename T>
void Lqr<T>::update_error_integral(const state_t& x) {
    m_q += m_Ag.template bottomLeftCorner<T::n, T::n>()*(x - m_r);
}

template<typename T>
typename Lqr<T>::input_t Lqr<T>::control_calculate(const state_t& x) {
    perform_value_iteration();
    input_t reduced_u = K()*x + Ki()*m_q;
    if (m_m == T::m) {
        return reduced_u;
    }

    input_t u = input_t::Zero();
    for (unsigned int i = 0, j = 0; i < T::m; ++i) {
        if (m_mask[i]) {
            u[i] = reduced_u[j++];
        }
    }
    update_error_integral(x);
    return u;
}

template<typename T>
typename Lqr<T>::input_t Lqr<T>::control_calculate(const state_t& x, const state_t& r) {
    // NOTE: This behaves differently than the update functions in Kalman.
    // Here, passing in 'r' also changes the stored reference. In Kalman,
    // passing 'Q' or 'R' in the update functions uses the argument only during
    // the current update, and the stored value is not changed.
    set_reference(r);
    return control_calculate(x);
}

template<typename T>
void Lqr<T>::perform_value_iteration() {
    if (!m_system.Ad().isApprox(m_Ag.template topLeftCorner<T::n, T::n>()) ||
            !m_system.Bd().isApprox(m_Bd)) {
        // check if system has changed
        m_Ag.template topLeftCorner<T::n, T::n>() = m_system.Ad();
        m_Bd = m_system.Bd();
        reduce_input_matrices();
        m_steady_state = false;
    }

    if (!m_steady_state) {
        augmented_lqr_gain_t K = m_Kg;
        augmented_state_cost_t P = m_Pg;
        for (unsigned int i = 0; i < m_horizon; ++i) {
            update_lqr_gain();
            update_horizon_cost();
        }
        if (K.isApprox(m_Kg) && P.isApprox(m_Pg)) {
            m_steady_state = true;
        }
    }
}

template<typename T>
void Lqr<T>::update_lqr_gain() {
    input_cost_t M;
    if (m_m == T::m) {
        M.noalias() = m_R + m_Bg.transpose()*m_Pg*m_Bg;
        m_Kg.noalias() = -M.fullPivHouseholderQr().solve(m_Bg.transpose()*m_Pg*m_Ag);
    } else {
        M.topLeftCorner(m_m, m_m).noalias() =
            m_Rr.topLeftCorner(m_m, m_m) + m_Bg.leftCols(m_m).transpose()*m_Pg*m_Bg.leftCols(m_m);
        m_Kg.topRows(m_m).noalias() = -M.topLeftCorner(m_m, m_m).fullPivHouseholderQr().solve(
                m_Bg.leftCols(m_m).transpose()*m_Pg*m_Ag);
    }
}

template<typename T>
void Lqr<T>::update_horizon_cost() {
    augmented_state_matrix_t M;
    if (m_m == T::m) {
        M.noalias() = m_Ag + m_Bg*m_Kg;
        m_Pg = m_Kg.transpose()*m_R*m_Kg + M.transpose()*m_Pg*M;
    } else {
        M.noalias() = m_Ag + m_Bg.leftCols(m_m)*m_Kg.topRows(m_m);
        m_Pg = m_Kg.topRows(m_m).transpose()*m_Rr.topLeftCorner(m_m, m_m)*m_Kg.topRows(m_m) +
            M.transpose()*m_Pg*M;
    }
    m_Pg.template topLeftCorner<T::n, T::n>() += m_Q;
    m_Pg.template bottomRightCorner<T::n, T::n>() += m_Qi;
}

template<typename T>
void Lqr<T>::set_control_mask() {
    // R must be positive definite. If any entry is 0.0, interpret as an
    // unavailable input and reduce the input matrix.
    m_mask = (m_R.diagonal().array() != 0.0).template cast<uint32_t>();
    m_m = m_mask.sum();
    reduce_input_matrices();
}

template<typename T>
void Lqr<T>::reduce_input_matrices() {
    m_Kg.setZero();
    m_Pg.setZero();
    m_Bg.setZero();
    if (m_m == T::m) {
        m_Bg.template topRows<T::n>() = m_Bd;
        m_Rr.setZero();
    } else {
        for (unsigned int i = 0, j = 0; i < T::m; ++i) {
            if (m_mask[i]) {
                m_Bg.col(j).template topRows<T::n>() = m_Bd.col(i);
                m_Rr.col(j++) = m_R.col(i);
            }
        }
        for (unsigned int i = 0, j = 0; i < T::m; ++i) {
            if (m_mask[i]) {
                m_Rr.row(j++) = m_Rr.row(i);
            }
        }
    }
}


} // namespace controller
