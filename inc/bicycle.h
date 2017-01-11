#pragma once
#include <unordered_map>
#include <utility>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/functional/hash.hpp>
#include "discrete_linear.h"

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
 * As C and D can be changed, the corresponding output fields may change and the
 * provided enum class output_index_t may longer be correct. It is the user's
 * responsibility to ensure correct index access.
 */

class Bicycle final : public DiscreteLinear<5, 2, 2, 2> {
    public:
        static constexpr unsigned int p = 4;
        using auxiliary_state_t = Eigen::Matrix<real_t, p, 1>;

        // state enum definitions
        enum class input_index_t: uint8_t {
            roll_torque = 0,
            steer_torque,
            number_of_types
        };
        enum class state_index_t: uint8_t {
            yaw_angle = 0,
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

        Bicycle(const second_order_matrix_t& M, const second_order_matrix_t& C1,
                const second_order_matrix_t& K0, const second_order_matrix_t& K2,
                real_t wheelbase, real_t trail, real_t steer_axis_tilt,
                real_t rear_wheel_radius, real_t front_wheel_radius,
                real_t v, real_t dt,
                const state_space_map_t* discrete_state_space_map = nullptr);
        Bicycle(const char* param_file, real_t v, real_t dt,
                const state_space_map_t* discrete_state_space_map = nullptr);
        Bicycle(real_t v, real_t dt,
                const state_space_map_t* discrete_state_space_map = nullptr);

        bool auxiliary_state_field(full_state_index_t field) const;

        virtual state_t update_state(const state_t& x, const input_t& u, const output_t& z) const override;
        virtual state_t update_state(const state_t& x, const input_t& u) const override;
        virtual output_t calculate_output(const state_t& x, const input_t& u) const override;
        virtual state_t update_state(const state_t& x) const override;
        virtual output_t calculate_output(const state_t& x) const override;
        state_t integrate_state(const state_t& x, const input_t& u, real_t dt) const;
        state_t integrate_state(const state_t& x, real_t dt) const;
        auxiliary_state_t update_auxiliary_state(const state_t& x, const auxiliary_state_t& x_aux) const;

        void set_v_dt(real_t v, real_t dt);
        void set_M(second_order_matrix_t& M, bool recalculate_state_space = true);
        void set_C1(second_order_matrix_t& C1, bool recalculate_state_space = true);
        void set_K0(second_order_matrix_t& K0, bool recalculate_state_space = true);
        void set_K2(second_order_matrix_t& K2, bool recalculate_state_space = true);
        void set_wheelbase(real_t w, bool recalculate_parameters = true);
        void set_trail(real_t c, bool recalculate_parameters = true);
        void set_steer_axis_tilt(real_t lambda, bool recalculate_parameters = true);
        void set_rear_wheel_radius(real_t rr, bool recalculate_moore_parameters = true);
        void set_front_wheel_radius(real_t rf, bool recalculate_moore_parameters = true);
        void set_C(const output_matrix_t& C);
        void set_D(const feedthrough_matrix_t& D);

        static constexpr state_space_map_key_t make_state_space_map_key(real_t v, real_t dt);
        bool discrete_state_space_lookup(const state_space_map_key_t& k);

        void set_state_space();
        void set_moore_parameters();

        real_t solve_constraint_pitch(const state_t& x, real_t guess) const;
        real_t calculate_handlebar_feedback_torque(const state_t& x, const input_t& u) const;

        // (pseudo) parameter accessors
        const state_matrix_t& A() const;
        const input_matrix_t& B() const;
        const output_matrix_t& C() const;
        const feedthrough_matrix_t& D() const;
        virtual const state_matrix_t& Ad() const override;
        virtual const input_matrix_t& Bd() const override;
        virtual const output_matrix_t& Cd() const override;
        virtual const feedthrough_matrix_t& Dd() const override;
        const second_order_matrix_t& M() const;
        const second_order_matrix_t& C1() const;
        const second_order_matrix_t& K0() const;
        const second_order_matrix_t& K2() const;
        const second_order_matrix_t& K() const;
        real_t wheelbase() const;
        real_t trail() const;
        real_t steer_axis_tilt() const;
        real_t rear_wheel_radius() const;
        real_t front_wheel_radius() const;
        real_t v() const;
        virtual real_t dt() const override;

        bool need_recalculate_state_space() const;
        bool need_recalculate_moore_parameters() const;

    private:
        // The full state matrix A is singular as yaw rate, and all other
        // states, are independent of yaw angle. As we discretize the continuous
        // state space, this is problematic for computation of Bd since we assume
        // that A is rarely singular.
        // The continuous time state space is discretized using the following property:
        //      [ A  B ]         [ Ad  Bd ]
        // exp( [ 0  0 ] * T ) = [  0   I ]
        using discretization_matrix_t = Eigen::Matrix<real_t, n + m, n + m>;

        real_t m_v; // parameterized forward speed
        real_t m_dt; // sampling time of discrete time system
        second_order_matrix_t m_M;
        second_order_matrix_t m_C1;
        second_order_matrix_t m_K0;
        second_order_matrix_t m_K2;
        second_order_matrix_t m_K; // g*K0 + v*v*K2
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

        state_matrix_t m_Ad;
        input_matrix_t m_Bd;

        static constexpr uint32_t m_dt_key_precision = 1000;
        static constexpr int32_t m_v_key_precision = 1000000;

        state_space_map_t const* m_discrete_state_space_map;

        /* Some steppers have internal state and so none have do_step() defined as const.
         * While internal state may be changed from multiple calls to do_step(), the
         * state is 'reset' when all calls to do_step() are completed and the integration
         * has completed, thus we can mark them as mutable.
         */
        using odeint_state_t = Eigen::Matrix<real_t, n + m, 1>;
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            odeint_state_t, real_t, odeint_state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_stepper;
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            state_t, real_t, state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_stepper_noinput;
        using odeint_auxiliary_state_t = Eigen::Matrix<real_t, p + n, 1>;
        mutable boost::numeric::odeint::runge_kutta_dopri5<
            odeint_auxiliary_state_t, real_t, odeint_auxiliary_state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_auxiliary_stepper;

        void set_parameters_from_file(const char* param_file);
        void initialize_state_space_matrices();
}; // class Bicycle

// define simple member functions using inline
inline void Bicycle::set_M(second_order_matrix_t& M, bool recalculate_state_space) {
    m_M = M;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}
inline void Bicycle::set_C1(second_order_matrix_t& C1, bool recalculate_state_space) {
    m_C1 = C1;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}
inline void Bicycle::set_K0(second_order_matrix_t& K0, bool recalculate_state_space) {
    m_K0 = K0;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}
inline void Bicycle::set_K2(second_order_matrix_t& K2, bool recalculate_state_space) {
    m_K2 = K2;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}
inline void Bicycle::set_wheelbase(real_t w, bool recalculate_parameters) {
    m_w = w;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}
inline void Bicycle::set_trail(real_t c, bool recalculate_parameters) {
    m_c = c;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}
inline void Bicycle::set_steer_axis_tilt(real_t lambda, bool recalculate_parameters) {
    m_lambda = lambda;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}
inline void Bicycle::set_rear_wheel_radius(real_t rr, bool recalculate_moore_parameters) {
    m_rr = rr;
    if (recalculate_moore_parameters) {
        set_moore_parameters();
    } else {
        m_recalculate_moore_parameters = true;
    }
}
inline void Bicycle::set_front_wheel_radius(real_t rf, bool recalculate_moore_parameters) {
    m_rf = rf;
    if (recalculate_moore_parameters) {
        set_moore_parameters();
    } else {
        m_recalculate_moore_parameters = true;
    }
}
inline void Bicycle::set_C(const output_matrix_t& C) {
    m_C = C;
}
inline void Bicycle::set_D(const feedthrough_matrix_t& D) {
    m_D = D;
}
inline constexpr Bicycle::state_space_map_key_t Bicycle::make_state_space_map_key(real_t v, real_t dt) {
    return state_space_map_key_t(m_dt_key_precision*dt, m_v_key_precision*v);
}
inline const Bicycle::state_matrix_t& Bicycle::A() const {
    return m_A;
}
inline const Bicycle::input_matrix_t& Bicycle::B() const {
    return m_B;
}
inline const Bicycle::output_matrix_t& Bicycle::C() const {
    return m_C;
}
inline const Bicycle::feedthrough_matrix_t& Bicycle::D() const {
    return m_D;
}
inline const Bicycle::state_matrix_t& Bicycle::Ad() const {
    return m_Ad;
}
inline const Bicycle::input_matrix_t& Bicycle::Bd() const {
    return m_Bd;
}
inline const Bicycle::output_matrix_t& Bicycle::Cd() const {
    return m_C;
}
inline const Bicycle::feedthrough_matrix_t& Bicycle::Dd() const {
    return m_D;
}
inline const Bicycle::second_order_matrix_t& Bicycle::M() const {
    return m_M;
}
inline const Bicycle::second_order_matrix_t& Bicycle::C1() const {
    return m_C1;
}
inline const Bicycle::second_order_matrix_t& Bicycle::K0() const {
    return m_K0;
}
inline const Bicycle::second_order_matrix_t& Bicycle::K2() const {
    return m_K2;
}
inline const Bicycle::second_order_matrix_t& Bicycle::K() const {
    return m_K;
}
inline real_t Bicycle::wheelbase() const {
    return m_w;
}
inline real_t Bicycle::trail() const {
    return m_c;
}
inline real_t Bicycle::steer_axis_tilt() const {
    return m_lambda;
}
inline real_t Bicycle::rear_wheel_radius() const {
    return m_rr;
}
inline real_t Bicycle::front_wheel_radius() const {
    return m_rf;
}
inline real_t Bicycle::v() const {
    return m_v;
}
inline real_t Bicycle::dt() const {
    return m_dt;
}
inline bool Bicycle::need_recalculate_state_space() const {
    return m_recalculate_state_space;
}
inline bool Bicycle::need_recalculate_moore_parameters() const {
    return m_recalculate_moore_parameters;
}
} // namespace model
