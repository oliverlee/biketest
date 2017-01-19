#pragma once
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include "linear.h"

namespace model {

/* State, Input, and Output Definitions
 * state: [yaw angle, roll angle, steer angle, roll rate, steer rate]
 * input: [roll torque, steer torque]
 * output: [yaw angle, steer angle]
 * auxiliary: [x rear contact, y rear contact, rear wheel angle, pitch angle]
 *
 * Note: 2 outputs are defined and are specified by the default C and D matrices,
 * however, C and D _can_ be set with member functions.
 *
 * As C and D can be changed, the corresponding output fields may change from
 * yaw angle and steer angle and the provided enum class output_index_t may
 * longer be correct. It is the user's responsibility to ensure correct
 * index access.
 */

class Bicycle : public Linear<5, 2, 2, 2> {
    public:
        static constexpr unsigned int p = 4;
        using auxiliary_state_t = Eigen::Matrix<real_t, p, 1>;

        /* state enum definitions */
        enum class input_index_t: uint8_t {
            roll_torque = 0,
            steer_torque,
            number_of_types
        };
        enum class state_index_t: uint8_t {
            yaw_angle = 0, /* yaw is included due it's linear relation to other state elements */
            roll_angle,
            steer_angle,
            roll_rate,
            steer_rate,
            number_of_types
        };
        enum class output_index_t: uint8_t {
            yaw_angle = 0,
            steer_angle,
            number_of_types
        };
        enum class auxiliary_state_index_t: uint8_t {
            x = 0,
            y,
            rear_wheel_angle,
            pitch_angle,
            number_of_types
        };
        enum class full_state_index_t: uint8_t {
            x = 0, /* always declare auxiliary state fields first */
            y,
            rear_wheel_angle,
            pitch_angle,
            yaw_angle,
            roll_angle,
            steer_angle,
            roll_rate,
            steer_rate,
            number_of_types
        };

        static bool is_auxiliary_state_field(full_state_index_t field);

        /* pure virtual state and output functions repeated */
        virtual state_t integrate_state(real_t t, const state_t& x, const input_t& u, const measurement_t& z) const override = 0;
        virtual output_t calculate_output(const state_t& x, const input_t& u) const override final;

        auxiliary_state_t integrate_auxiliary_state(const state_t& x, const auxiliary_state_t& x_aux, real_t t) const;

        virtual void set_v(real_t v); /* this function _always_ recalculates state space */
        virtual void set_M(second_order_matrix_t& M, bool recalculate_state_space);
        virtual void set_C1(second_order_matrix_t& C1, bool recalculate_state_space);
        virtual void set_K0(second_order_matrix_t& K0, bool recalculate_state_space);
        virtual void set_K2(second_order_matrix_t& K2, bool recalculate_state_space);
        virtual void set_wheelbase(real_t w, bool recalculate_parameters);
        virtual void set_trail(real_t c, bool recalculate_parameters);
        virtual void set_steer_axis_tilt(real_t lambda, bool recalculate_parameters);
        virtual void set_rear_wheel_radius(real_t rr, bool recalculate_parameters);
        virtual void set_front_wheel_radius(real_t rf, bool recalculate_parameter);
        void set_C(const output_matrix_t& C);
        void set_D(const feedthrough_matrix_t& D);

        virtual void set_state_space();
        void set_moore_parameters();

        real_t solve_constraint_pitch(real_t roll_angle, real_t steer_angle, real_t guess) const;

        // (pseudo) parameter accessors
        virtual const state_matrix_t& A() const override final;
        virtual const input_matrix_t& B() const override final;
        virtual const output_matrix_t& C() const override final;
        virtual const feedthrough_matrix_t& D() const override final;
        const second_order_matrix_t& M() const;
        const second_order_matrix_t& C1() const;
        const second_order_matrix_t& K0() const;
        const second_order_matrix_t& K2() const;
        real_t wheelbase() const;
        real_t trail() const;
        real_t steer_axis_tilt() const;
        real_t rear_wheel_radius() const;
        real_t front_wheel_radius() const;
        real_t v() const;

        bool need_recalculate_state_space() const;
        bool need_recalculate_moore_parameters() const;

        // utility class static member functions
        virtual state_t normalize_state(const state_t& x) const override final;
        virtual output_t normalize_output(const output_t& y) const override final;
        auxiliary_state_t normalize_auxiliary_state(const auxiliary_state_t& x_aux) const;

    protected:
        real_t m_v; // parameterized forward speed
        second_order_matrix_t m_M;
        second_order_matrix_t m_C1;
        second_order_matrix_t m_K0;
        second_order_matrix_t m_K2;
        real_t m_w;
        real_t m_c;
        real_t m_lambda;
        real_t m_rr;
        real_t m_rf;
        real_t m_d1; // Moore parameter. Luke calls this cR.
        real_t m_d2; // Moore parameter. Luke calls this ls.
        real_t m_d3; // Moore parameter. Luke calls this cF.

        bool m_recalculate_state_space;
        bool m_recalculate_moore_parameters;

        Eigen::LLT<second_order_matrix_t> m_M_llt;
        state_matrix_t m_A;
        input_matrix_t m_B;
        output_matrix_t m_C;
        feedthrough_matrix_t m_D;

        /*
         * Some steppers have internal state and so none have do_step() defined as const.
         * While internal state may be changed from multiple calls to do_step(), the
         * state is 'reset' when all calls to do_step() are completed and the integration
         * has completed, thus we can mark them as mutable.
         */
        using full_state_t = Eigen::Matrix<real_t, p + n, 1>;
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            full_state_t, real_t, full_state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_auxiliary_stepper;

        Bicycle(const second_order_matrix_t& M, const second_order_matrix_t& C1,
                const second_order_matrix_t& K0, const second_order_matrix_t& K2,
                real_t wheelbase, real_t trail, real_t steer_axis_tilt,
                real_t rear_wheel_radius, real_t front_wheel_radius,
                real_t v);
        Bicycle(const char* param_file, real_t v);
        Bicycle(real_t v);
        void set_parameters_from_file(const char* param_file);
}; // class Bicycle

} // namespace model
