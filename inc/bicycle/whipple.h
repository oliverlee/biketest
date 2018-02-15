#pragma once
#include "bicycle.h"
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

namespace model {

class BicycleWhipple final : public Bicycle {
    public:
        BicycleWhipple(const second_order_matrix_t& M, const second_order_matrix_t& C1,
                const second_order_matrix_t& K0, const second_order_matrix_t& K2,
                real_t wheelbase, real_t trail, real_t steer_axis_tilt,
                real_t rear_wheel_radius, real_t front_wheel_radius,
                real_t v, real_t dt);
        BicycleWhipple(const char* param_file, real_t v, real_t dt);
        BicycleWhipple(real_t v, real_t dt);

        virtual state_t update_state(const state_t& x, const input_t& u = input_t::Zero(), const measurement_t& z = measurement_t::Zero()) const override;
        virtual full_state_t integrate_full_state(const full_state_t& xf, const input_t& u, real_t t, const measurement_t& z = measurement_t::Zero()) const override;
        virtual state_t integrate_state(const state_t& x, const input_t& u, real_t t) const;

        virtual void set_state_space() override;

    private:
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

} // namespace model
