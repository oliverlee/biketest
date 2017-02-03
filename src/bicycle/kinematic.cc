#include "bicycle/kinematic.h"
#include "constants.h"

namespace model {
BicycleKinematic::BicycleKinematic(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v, real_t dt) :
    Bicycle(M, C1, K0, K2, wheelbase, trail, steer_axis_tilt, rear_wheel_radius, front_wheel_radius, v, dt) {
    set_K();
}

BicycleKinematic::BicycleKinematic(const char* param_file, real_t v, real_t dt) :
    Bicycle(param_file, v, dt) {
    set_K();
}

BicycleKinematic::BicycleKinematic(real_t v, real_t dt) :
    Bicycle(v, dt) {
    set_K();
}

BicycleKinematic::state_t BicycleKinematic::update_state(const BicycleKinematic::state_t& x, const BicycleKinematic::input_t& u, const BicycleKinematic::measurement_t& z) const {
    /*
     * Simplified equations of motion are used to simulate the bicycle dynamics.
     * Roll/steer rate and acceleration terms are ignored resulting in:
     *          (g*K0 + v^2*K2) [phi  ] = [T_phi  ]
     *                          [delta] = [T_delta]
     *
     */
    (void)u;
    const real_t yaw_angle_measurement = get_output_element(z, output_index_t::yaw_angle);
    const real_t steer_angle_measurement = get_output_element(z, output_index_t::steer_angle);
    const real_t next_roll = -m_K(0, 1)/m_K(0, 0) * steer_angle_measurement;

    state_t next_x = state_t::Zero();
    set_state_element(next_x, state_index_t::yaw_angle, yaw_angle_measurement);
    set_state_element(next_x, state_index_t::steer_rate,
            (steer_angle_measurement - get_state_element(x, state_index_t::steer_angle))/m_dt);
    set_state_element(next_x, state_index_t::roll_rate,
            (next_roll - get_state_element(x, state_index_t::roll_angle))/m_dt);
    set_state_element(next_x, state_index_t::steer_angle, steer_angle_measurement);
    set_state_element(next_x, state_index_t::roll_angle, next_roll);
    return next_x;
}

BicycleKinematic::full_state_t BicycleKinematic::integrate_full_state(const BicycleKinematic::full_state_t& xf, const BicycleKinematic::input_t& u, real_t t) const {
    /*
     * As this class is already a simplification, we integrate the auxiliary state part separately, using the state at
     * the previous time. After, integration of the auxiliary state, the dynamic state is updated.
     */
    measurement_t z = measurement_t::Zero(); // FIXME need z as an argument

    static constexpr auto x_index = index(full_state_index_t::x);
    static constexpr auto y_index = index(full_state_index_t::y);
    static constexpr auto rear_wheel_index = index(full_state_index_t::rear_wheel_angle);
    static constexpr auto pitch_index = index(full_state_index_t::pitch_angle);

    const real_t v = m_v;
    const real_t rr = m_rr;
    const real_t yaw_angle = get_full_state_element(xf, full_state_index_t::yaw_angle);

    auxiliary_state_t x_aux_out = get_auxiliary_state_part(xf);

    m_stepper.do_step([v, rr, yaw_angle](const auxiliary_state_t& x, auxiliary_state_t& dxdt, const real_t t) -> void {
            (void)t; // system is time-independent;
            (void)x;

            // auxiliary state fields only
            dxdt[x_index] = v*std::cos(yaw_angle);
            dxdt[y_index] = v*std::sin(yaw_angle);
            dxdt[rear_wheel_index] = -v/rr;
            dxdt[pitch_index] = 0; // pitch angle is not integrated and must be obtained using solve_pitch_constraint()
            }, x_aux_out, static_cast<real_t>(0), t);

    const state_t x_out = update_state(get_state_part(xf), u, z);
    return make_full_state(x_aux_out, x_out);
}

void BicycleKinematic::set_state_space() {
    // FIXME this functionality is the same as in BicycleWhipple (well computation of the discrete state space)
    // Maybe we can just set K here?
    set_K();
}

void BicycleKinematic::set_K() {
    m_K = constants::g*m_K0 + m_v*m_v*m_K2;
}


} // namespace model