#pragma once
#include "bicycle.h"

namespace model {

class BicycleWhipple final : public Bicycle {
    public:
        BicycleWhipple(const second_order_matrix_t& M, const second_order_matrix_t& C1,
                const second_order_matrix_t& K0, const second_order_matrix_t& K2,
                real_t wheelbase, real_t trail, real_t steer_axis_tilt,
                real_t rear_wheel_radius, real_t front_wheel_radius,
                real_t v);
        BicycleWhipple(const char* param_file, real_t v);
        BicycleWhipple(real_t v);

        virtual state_t integrate_state(real_t t, const state_t& x, const input_t& u = input_t::Zero(), const measurement_t& z = measurement_t::Zero()) const override;

    private:
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

} // namespace model
