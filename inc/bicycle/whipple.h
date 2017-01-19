#pragma once
#include <unordered_map>
#include <utility>
#include "bicycle.h"
#include <boost/functional/hash.hpp>

namespace model {

class BicycleWhipple final : public Bicycle {
    public:
        /*
         * We normally treat speed v as a double/float. However to allow for constant
         * time lookup along with quickly finding a key 'near' the requested
         * one, we convert speeds to a fixed precision integer.
         *
         * For example, if six digits after the decimal defines the precision used:
         * v = 6.024262 -> 6024262
         *
         * And the same is done with sample time dt and if microsecond precision is used:
         * dt = 0.005 -> 5000
         *
         * Keys are defined as a pair:
         * (round(m_dt_key_precision*dt), round(m_v_key_precision*v)).
         * Where precision is set with m_dt_key_precision and m_v_key_precision.
         */
        using state_space_map_key_t = const std::pair<uint32_t, int32_t>;
        using state_space_map_value_t = const std::pair<state_matrix_t, input_matrix_t>;
        using state_space_map_t = std::unordered_map<state_space_map_key_t,
              state_space_map_value_t, boost::hash<state_space_map_key_t>>;

        BicycleWhipple(const second_order_matrix_t& M, const second_order_matrix_t& C1,
                const second_order_matrix_t& K0, const second_order_matrix_t& K2,
                real_t wheelbase, real_t trail, real_t steer_axis_tilt,
                real_t rear_wheel_radius, real_t front_wheel_radius,
                real_t v, real_t dt,
                const state_space_map_t* discrete_state_space_map = nullptr);
        BicycleWhipple(const char* param_file, real_t v, real_t dt,
                const state_space_map_t* discrete_state_space_map = nullptr);
        BicycleWhipple(real_t v, real_t dt,
                const state_space_map_t* discrete_state_space_map = nullptr);

        virtual state_t update_state(const state_t& x, const input_t& u = input_t::Zero(), const measurement_t& z = measurement_t::Zero()) const override;
        virtual output_t calculate_output(const state_t& x, const input_t& u = input_t::Zero()) const override;

        state_t integrate_state(const state_t& x, const input_t& u, real_t t) const;

        virtual void set_state_space() override;

        static constexpr state_space_map_key_t make_state_space_map_key(real_t v, real_t dt);
        bool discrete_state_space_lookup(real_t v, real_t dt);

        // (pseudo) parameter accessors
        virtual const state_matrix_t& Ad() const override;
        virtual const input_matrix_t& Bd() const override;
        virtual const output_matrix_t& Cd() const override;
        virtual const feedthrough_matrix_t& Dd() const override;

    private:
        state_matrix_t m_Ad;
        input_matrix_t m_Bd;

        static constexpr uint32_t m_dt_key_precision = 1000;
        static constexpr int32_t m_v_key_precision = 1000000;

        const state_space_map_t* const m_discrete_state_space_map;

        /*
         * Some steppers have internal state and so none have do_step() defined as const.
         * While internal state may be changed from multiple calls to do_step(), the
         * state is 'reset' when all calls to do_step() are completed and the integration
         * has completed, thus we mark them as mutable as we do not perceive the internal
         * state change of the stepper.
         */
        using odeint_state_t = Eigen::Matrix<real_t, n + m, 1>;
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            odeint_state_t, real_t, odeint_state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_stepper;
};

inline constexpr BicycleWhipple::state_space_map_key_t BicycleWhipple::make_state_space_map_key(real_t v, real_t dt) {
    return state_space_map_key_t(m_dt_key_precision*dt, m_v_key_precision*v);
}

} // namespace model
