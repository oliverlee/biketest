#include <cmath>
#include "bicycle/whipple.h"

namespace model {
BicycleWhipple::BicycleWhipple(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
        , const state_space_map_t* discrete_state_space_map
#endif // BICYCLE_USE_STATE_SPACE_MAP
        ) :
    Bicycle(M, C1, K0, K2, wheelbase, trail, steer_axis_tilt, rear_wheel_radius, front_wheel_radius, v, dt)
#if BICYCLE_USE_STATE_SPACE_MAP
    , m_discrete_state_space_map(discrete_state_space_map)
#endif // BICYCLE_USE_STATE_SPACE_MAP
    { }

BicycleWhipple::BicycleWhipple(const char* param_file, real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
        , const state_space_map_t* discrete_state_space_map
#endif // BICYCLE_USE_STATE_SPACE_MAP
        ) :
    Bicycle(param_file, v, dt)
#if BICYCLE_USE_STATE_SPACE_MAP
    , m_discrete_state_space_map(discrete_state_space_map)
#endif // BICYCLE_USE_STATE_SPACE_MAP
    { }

BicycleWhipple::BicycleWhipple(real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
        , const state_space_map_t* discrete_state_space_map
#endif // BICYCLE_USE_STATE_SPACE_MAP
        ) :
    Bicycle(v, dt)
#if BICYCLE_USE_STATE_SPACE_MAP
    , m_discrete_state_space_map(discrete_state_space_map)
#endif // BICYCLE_USE_STATE_SPACE_MAP
    { }

BicycleWhipple::state_t BicycleWhipple::update_state(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u, const BicycleWhipple::measurement_t& z) const {
    (void)z;
    return m_Ad*x + m_Bd*u;
}

BicycleWhipple::full_state_t BicycleWhipple::integrate_full_state(const BicycleWhipple::full_state_t& xf, const BicycleWhipple::input_t& u, real_t t, const BicycleWhipple::measurement_t& z) const {
    (void)z;
    static constexpr auto x_index = index(full_state_index_t::x);
    static constexpr auto y_index = index(full_state_index_t::y);
    static constexpr auto rear_wheel_index = index(full_state_index_t::rear_wheel_angle);
    static constexpr auto pitch_index = index(full_state_index_t::pitch_angle);
    static constexpr auto yaw_index = index(full_state_index_t::yaw_angle);

    const real_t v = m_v;
    const real_t rr = m_rr;
    const state_matrix_t& A = m_A;
    const Eigen::LLT<second_order_matrix_t>& M_llt = m_M_llt;

    full_state_t xout = xf;

    m_stepper.do_step([&A, &M_llt, &u, v, rr](const full_state_t& x, full_state_t& dxdt, const real_t t) -> void {
            (void)t; // system is time-independent

            // auxiliary state fields first
            dxdt[x_index] = v*std::cos(x[yaw_index]);
            dxdt[y_index] = v*std::sin(x[yaw_index]);
            dxdt[rear_wheel_index] = -v/rr;
            dxdt[pitch_index] = 0; // pitch angle is not integrated and must be obtained using solve_pitch_constraint()

            // state fields
            dxdt.tail<n>() = A*x.tail<n>();
            // Normally we would write dxdt = A*x + B*u but we prefer not to
            // use the matrix inverse unless absolutely necessary when computing.
            // As B = [   0  ], the product Bu = [      0   ]
            //        [ M^-1 ]                   [ M^-1 * u ]
            dxdt.tail<o>() += M_llt.solve(u);
            }, xout, static_cast<real_t>(0), t); // newly obtained state written in place
    return xout;
}

BicycleWhipple::state_t BicycleWhipple::integrate_state(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u, real_t t) const {
    state_t xout = x;

    const state_matrix_t& A = m_A;
    const Eigen::LLT<second_order_matrix_t>& M_llt = m_M_llt;

    m_stepper_state.do_step([&A, &M_llt, &u](const state_t& x, state_t& dxdt, const real_t t) -> void {
            (void)t; // system is time-independent
            dxdt = A*x;
            dxdt.tail<o>() += M_llt.solve(u);
            }, xout, static_cast<real_t>(0), t); // newly obtained state written in place
    return xout;
}

void BicycleWhipple::set_state_space() {
    /*
     * System state space is parameterized by forward speed v so this function
     * is always called after setting v. This function calculates the state
     * space matrices and additionally calculates discrete time state space if
     * sampling time is nonzero.
     *
     * The Whipple class allows usage of a discrete-time state space lookup map.
     * As the discrete-time state space computation will be calculated in the
     * base class, we first check if the lookup succeeds, and if so, set m_dt
     * to zero to skip the discrete-time computation.
     *
     * Should we even keep this? It seems extremely unlikely to ever be used given
     * the time to calculate is quite small.
     */
#if BICYCLE_USE_STATE_SPACE_MAP
    real_t dt = static_cast<real_t>(0);
    if (in_discrete_state_space_map(m_v, m_dt)) {
        dt = m_dt;
        m_dt = static_cast<real_t>(0);
    }
#endif // BICYCLE_USE_STATE_SPACE_MAP
    Bicycle::set_state_space();
#if BICYCLE_USE_STATE_SPACE_MAP
    if (dt != static_cast<real_t>(0)) {
        m_dt = dt;
        set_discrete_state_space_from_map(m_v, m_dt);
    }
#endif // BICYCLE_USE_STATE_SPACE_MAP
}

#if BICYCLE_USE_STATE_SPACE_MAP
bool BicycleWhipple::in_discrete_state_space_map(real_t v, real_t dt) {
    if (m_discrete_state_space_map == nullptr) {
        return false;
    }

    state_space_map_key_t k = make_state_space_map_key(v, dt);
    auto search = m_discrete_state_space_map->find(k);
    if (search == m_discrete_state_space_map->end()) {
        return false;
    }

    return true;
}

void BicycleWhipple::set_discrete_state_space_from_map(real_t v, real_t dt) {
    if (m_discrete_state_space_map != nullptr) {
        state_space_map_key_t k = make_state_space_map_key(v, dt);
        auto search = m_discrete_state_space_map->find(k);
        if (search != m_discrete_state_space_map->end()) {
            // discrete state space matrices Ad, Bd have been provided for speed v, sample time dt.
            m_Ad = search->second.first;
            m_Bd = search->second.second;
            return;
        }
    }
}
#endif // BICYCLE_USE_STATE_SPACE_MAP

} // namespace model
