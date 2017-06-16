#pragma once
#if BICYCLE_USE_STATE_SPACE_MAP
#include <unordered_map>
#include <utility>
#endif // BICYCLE_USE_STATE_SPACE_MAP
#include "bicycle.h"
#if BICYCLE_USE_STATE_SPACE_MAP
#include <boost/functional/hash.hpp>
#endif // BICYCLE_USE_STATE_SPACE_MAP
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

namespace model {

class BicycleWhipple final : public Bicycle<> {
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
#if BICYCLE_USE_STATE_SPACE_MAP
        using state_space_map_key_t = const std::pair<uint32_t, int32_t>;
        using state_space_map_value_t = const std::pair<state_matrix_t, input_matrix_t>;
        using state_space_map_t = std::unordered_map<state_space_map_key_t,
              state_space_map_value_t, boost::hash<state_space_map_key_t>>;
#endif // BICYCLE_USE_STATE_SPACE_MAP

        BicycleWhipple(const second_order_matrix_t& M, const second_order_matrix_t& C1,
                const second_order_matrix_t& K0, const second_order_matrix_t& K2,
                real_t wheelbase, real_t trail, real_t steer_axis_tilt,
                real_t rear_wheel_radius, real_t front_wheel_radius,
                real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
                , const state_space_map_t* discrete_state_space_map = nullptr
#endif // BICYCLE_USE_STATE_SPACE_MAP
                );
        BicycleWhipple(const char* param_file, real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
                , const state_space_map_t* discrete_state_space_map = nullptr
#endif // BICYCLE_USE_STATE_SPACE_MAP
                );
        BicycleWhipple(real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
                , const state_space_map_t* discrete_state_space_map = nullptr
#endif // BICYCLE_USE_STATE_SPACE_MAP
                );

        virtual state_t update_state(const state_t& x, const input_t& u = input_t::Zero(), const measurement_t& z = measurement_t::Zero()) const override;
        virtual full_state_t integrate_full_state(const full_state_t& xf, const input_t& u, real_t t, const measurement_t& z = measurement_t::Zero()) const override;
        virtual state_t integrate_state(const state_t& x, const input_t& u, real_t t) const;

        virtual void set_state_space() override;

#if BICYCLE_USE_STATE_SPACE_MAP
        static constexpr state_space_map_key_t make_state_space_map_key(real_t v, real_t dt);
        bool in_discrete_state_space_map(real_t v, real_t dt);
        void set_discrete_state_space_from_map(real_t v, real_t dt);
#endif // BICYCLE_USE_STATE_SPACE_MAP

    private:
#if BICYCLE_USE_STATE_SPACE_MAP
        static constexpr uint32_t m_dt_key_precision = 1000;
        static constexpr int32_t m_v_key_precision = 1000000;

        const state_space_map_t* const m_discrete_state_space_map;
#endif // BICYCLE_USE_STATE_SPACE_MAP

        /*
         * Some steppers have internal state and so none have do_step() defined as const.
         * While internal state may be changed from multiple calls to do_step(), the
         * state is 'reset' when all calls to do_step() are completed and the integration
         * has completed, thus we mark them as mutable as we do not perceive the internal
         * state change of the stepper.
         */
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            full_state_t, real_t, full_state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_stepper;
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            state_t, real_t, state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_stepper_state;
};

#if BICYCLE_USE_STATE_SPACE_MAP
inline constexpr BicycleWhipple::state_space_map_key_t BicycleWhipple::make_state_space_map_key(real_t v, real_t dt) {
    return state_space_map_key_t(m_dt_key_precision*dt, m_v_key_precision*v);
}
#endif // BICYCLE_USE_STATE_SPACE_MAP

} // namespace model
