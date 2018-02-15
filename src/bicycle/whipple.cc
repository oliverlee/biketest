#include <cmath>
#include "bicycle/whipple.h"

namespace {
    template <typename E>
    constexpr uint8_t index(E e) {
        return static_cast<uint8_t>(e);
    }
} // namespace

namespace model {
BicycleWhipple::BicycleWhipple(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v, real_t dt) :
    Bicycle(M, C1, K0, K2, wheelbase, trail, steer_axis_tilt, rear_wheel_radius, front_wheel_radius, v, dt)
    { }

BicycleWhipple::BicycleWhipple(const char* param_file, real_t v, real_t dt) :
    Bicycle(param_file, v, dt)
    { }

BicycleWhipple::BicycleWhipple(real_t v, real_t dt) :
    Bicycle(v, dt)
    { }

BicycleWhipple::state_t BicycleWhipple::update_state(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u, const BicycleWhipple::measurement_t& z) const {
    (void)z;
#if !defined(BICYCLE_NO_DISCRETIZATION)
    // This simply calls integrate state with the integration time set to m_dt
    // if discretization has been disabled.
    return integrate_state(x, u, m_dt);
#else
    return m_Ad*x + m_Bd*u;
#endif
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
     */
    Bicycle::set_state_space();
}

} // namespace model
