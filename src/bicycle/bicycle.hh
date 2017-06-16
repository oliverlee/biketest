#include <array>
#include <cmath>
#include <fstream>
#include <type_traits>
#include <unsupported/Eigen/MatrixFunctions>
#include "parameters.h"

namespace model {
    template <typename E>
    constexpr typename std::underlying_type<E>::type index(E e) {
        return static_cast<typename std::underlying_type<E>::type>(e);
    }

    template <typename OutputIndexEnum, typename T, typename I>
    constexpr void set_element(T& x, I field, real_t value) {
        static_assert(
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::state_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::state_index_t>::value) ||
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::auxiliary_state_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::auxiliary_state_index_t>::value) ||
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::full_state_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::full_state_index_t>::value) ||
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::input_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::input_index_t>::value) ||
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::output_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::output_index_t>::value),
                "Invalid index type for given state type.");
        x[index(field)] = value;
    }

    template <typename OutputIndexEnum, typename T, typename I>
    constexpr real_t get_element(const T& x, I field) {
        static_assert(
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::state_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::state_index_t>::value) ||
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::auxiliary_state_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::auxiliary_state_index_t>::value) ||
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::full_state_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::full_state_index_t>::value) ||
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::input_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::input_index_t>::value) ||
                (std::is_same<T, typename Bicycle<OutputIndexEnum>::output_t>::value && std::is_same<I, typename Bicycle<OutputIndexEnum>::output_index_t>::value),
                "Invalid index type for given state type.");
        return x[index(field)];
    }

    template <typename OutputIndexEnum, typename T, typename I>
    constexpr void mod_two_pi_element(T& x, I field) {
        const real_t modded_value = std::fmod(get_element<OutputIndexEnum>(x, field), constants::two_pi);
        set_element<OutputIndexEnum>(x, field, modded_value);
    }

#if !defined(NDEBUG)
    const real_t discretization_precision = Eigen::NumTraits<real_t>::dummy_precision();
#endif

template <typename OutputIndexEnum>
bool Bicycle<OutputIndexEnum>::is_auxiliary_state_field(full_state_index_t field) {
    return index(field) < index(auxiliary_state_index_t::number_of_types);
}

template <typename OutputIndexEnum>
typename Bicycle<OutputIndexEnum>::full_state_t Bicycle<OutputIndexEnum>::make_full_state(const auxiliary_state_t& aux, const state_t& x) {
    full_state_t xf;
    xf << aux, x;
    return xf;
}

template <typename OutputIndexEnum>
typename Bicycle<OutputIndexEnum>::auxiliary_state_t Bicycle<OutputIndexEnum>::get_auxiliary_state_part(const full_state_t& xf) {
    return xf.template head<p>();
}

template <typename OutputIndexEnum>
typename Bicycle<OutputIndexEnum>::state_t Bicycle<OutputIndexEnum>::get_state_part(const full_state_t& xf) {
    return xf.template tail<n>();
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_state_element(state_t& x, state_index_t field, real_t value) {
    set_element<OutputIndexEnum>(x, field, value);
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_auxiliary_state_element(auxiliary_state_t& x, auxiliary_state_index_t field, real_t value) {
    set_element<OutputIndexEnum>(x, field, value);
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_full_state_element(full_state_t& x, full_state_index_t field, real_t value) {
    set_element<OutputIndexEnum>(x, field, value);
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_input_element(input_t& x, input_index_t field, real_t value) {
    set_element<OutputIndexEnum>(x, field, value);
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_output_element(output_t& x, output_index_t field, real_t value) {
    set_element<OutputIndexEnum>(x, field, value);
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::get_state_element(const state_t& x, state_index_t field) {
    return get_element<OutputIndexEnum>(x, field);
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::get_auxiliary_state_element(const auxiliary_state_t& x, auxiliary_state_index_t field) {
    return get_element<OutputIndexEnum>(x, field);
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::get_full_state_element(const full_state_t& x, full_state_index_t field) {
    return get_element<OutputIndexEnum>(x, field);
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::get_input_element(const input_t& x, input_index_t field) {
    return get_element<OutputIndexEnum>(x, field);
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::get_output_element(const output_t& x, output_index_t field) {
    return get_element<OutputIndexEnum>(x, field);
}

template <typename OutputIndexEnum>
typename Bicycle<OutputIndexEnum>::output_t Bicycle<OutputIndexEnum>::calculate_output(const Bicycle<OutputIndexEnum>::state_t& x, const Bicycle<OutputIndexEnum>::input_t& u) const {
    return m_C*x + m_D*u;
}


template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_v_dt(real_t v, real_t dt) {
    m_v = v;
    m_dt = dt;

    set_state_space();
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_M(second_order_matrix_t& M, bool recalculate_state_space) {
    m_M = M;
    m_M_llt.compute(M);
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_C1(second_order_matrix_t& C1, bool recalculate_state_space) {
    m_C1 = C1;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_K0(second_order_matrix_t& K0, bool recalculate_state_space) {
    m_K0 = K0;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_K2(second_order_matrix_t& K2, bool recalculate_state_space) {
    m_K2 = K2;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_wheelbase(real_t w, bool recalculate_parameters) {
    m_w = w;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_trail(real_t c, bool recalculate_parameters) {
    m_c = c;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_steer_axis_tilt(real_t lambda, bool recalculate_parameters) {
    m_lambda = lambda;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_rear_wheel_radius(real_t rr, bool recalculate_moore_parameters) {
    m_rr = rr;
    if (recalculate_moore_parameters) {
        set_moore_parameters();
    } else {
        m_recalculate_moore_parameters = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_front_wheel_radius(real_t rf, bool recalculate_moore_parameters) {
    m_rf = rf;
    if (recalculate_moore_parameters) {
        set_moore_parameters();
    } else {
        m_recalculate_moore_parameters = true;
    }
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_C(const output_matrix_t& C) {
    m_C = C;
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_D(const feedthrough_matrix_t& D) {
    m_D = D;
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_state_space() {
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::yaw_angle) == 0,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::roll_angle) == 1,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::steer_angle) == 2,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::roll_rate) == 3,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::steer_rate) == 4,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::number_of_types) == 5,
        "Invalid underlying value for state index element");
    /*
     * q = [roll, steer]', q_d = [roll_rate, steer_rate]'
     * x = [yaw, q, q_d] = [yaw, roll, steer, roll_rate, steer_rate]
     * u = [T_roll, T_steer]
     *
     * M*q_dd + v*C1*q_d + (g*K0 + v^2*K2)*q = u
     * yaw_rate = cos(lambda)/w * (v*steer + c*steer_rate)
     *
     * x_d = [ 0                      a           b]*x + [   0]*u
     *       [ 0                      0           I]     [   0]
     *       [ 0  -M^-1*(g*K0 + v^2*K2)  -M^-1*v*C1]     [M^-1]
     *
     * a = [0, v*cos(lambda)/w]
     * b = [0, c*cos(lambda)/w]
     *
     * As M is positive definite, we use the Cholesky decomposition in solving the linear system
     *
     * If states change, we need to reformulate the state space matrix equations.
     */
    m_A(0, 2) = m_v * std::cos(m_lambda) / m_w; /* steer angle component of yaw rate */
    m_A(0, 4) = m_c * std::cos(m_lambda) / m_w; /* steer rate component of yaw rate */
    m_A.block<o, o>(1, 3).setIdentity();
    m_A.block<o, o>(3, 1) = -m_M_llt.solve(constants::g*m_K0 + m_v*m_v*m_K2);
    m_A.bottomRightCorner<o, o>() = -m_M_llt.solve(m_v*m_C1);
    m_B.bottomRows<o>() = m_M.inverse();
    m_recalculate_state_space = false;

    set_discrete_state_space();
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_discrete_state_space() {
    m_Ad.setZero();
    m_Bd.setZero();

    if (m_dt == static_cast<real_t>(0)) { // discrete time state does not change
        m_Ad.setIdentity();
        m_Bd.setZero();
    } else {
        /*
         * The full state matrix A is singular as yaw rate, and all other
         * states, are independent of yaw angle. As we discretize the continuous
         * state space, this is problematic for computation of Bd since we assume
         * that A is rarely singular.
         * The continuous time state space is discretized using the following property:
         *      [ A  B ]         [ Ad  Bd ]
         * exp( [ 0  0 ] * T ) = [  0   I ]
         */
        using discretization_matrix_t = Eigen::Matrix<real_t, n + m, n + m>;
        discretization_matrix_t AT = discretization_matrix_t::Zero();
        AT.topLeftCorner<n, n>() = m_A;
        AT.topRightCorner<n, m>() = m_B;
        AT *= m_dt;

        discretization_matrix_t T = AT.exp();
        assert((T.bottomLeftCorner<m, n>().isZero(discretization_precision)) &&
               (T.bottomRightCorner<m, m>().isIdentity(discretization_precision)));
        m_Ad = T.topLeftCorner<n, n>();
        m_Bd = T.topRightCorner<n, m>();
    }
}

/* set d1, d2, d3 used in pitch constraint calculation */
template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_moore_parameters() {
    m_d1 = std::cos(m_lambda)*(m_c + m_w - m_rr*std::tan(m_lambda));
    m_d3 = -std::cos(m_lambda)*(m_c - m_rf*std::tan(m_lambda));
    m_d2 = (m_rr + m_d1*std::sin(m_lambda) - m_rf + m_d3*std::sin(m_lambda)) / std::cos(m_lambda);
    m_recalculate_moore_parameters = false;
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::solve_constraint_pitch(real_t roll_angle, real_t steer_angle,
                                                        real_t guess, size_t max_iterations) const {
    return solve_constraint_pitch(roll_angle, steer_angle, guess, max_iterations,
                                  m_d1, m_d2, m_d3, m_rr, m_rf);
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::state_matrix_t& Bicycle<OutputIndexEnum>::Ad() const {
    return m_Ad;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::input_matrix_t& Bicycle<OutputIndexEnum>::Bd() const {
    return m_Bd;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::output_matrix_t& Bicycle<OutputIndexEnum>::Cd() const {
    return C();
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::feedthrough_matrix_t& Bicycle<OutputIndexEnum>::Dd() const {
    return D();
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::state_matrix_t& Bicycle<OutputIndexEnum>::A() const {
    return m_A;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::input_matrix_t& Bicycle<OutputIndexEnum>::B() const {
    return m_B;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::output_matrix_t& Bicycle<OutputIndexEnum>::C() const {
    return m_C;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::feedthrough_matrix_t& Bicycle<OutputIndexEnum>::D() const {
    return m_D;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::second_order_matrix_t& Bicycle<OutputIndexEnum>::M() const {
    return m_M;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::second_order_matrix_t& Bicycle<OutputIndexEnum>::C1() const {
    return m_C1;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::second_order_matrix_t& Bicycle<OutputIndexEnum>::K0() const {
    return m_K0;
}

template <typename OutputIndexEnum>
const typename Bicycle<OutputIndexEnum>::second_order_matrix_t& Bicycle<OutputIndexEnum>::K2() const {
    return m_K2;
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::wheelbase() const {
    return m_w;
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::trail() const {
    return m_c;
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::steer_axis_tilt() const {
    return m_lambda;
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::rear_wheel_radius() const {
    return m_rr;
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::front_wheel_radius() const {
    return m_rf;
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::v() const {
    return m_v;
}

template <typename OutputIndexEnum>
real_t Bicycle<OutputIndexEnum>::dt() const {
    return m_dt;
}

template <typename OutputIndexEnum>
bool Bicycle<OutputIndexEnum>::need_recalculate_state_space() const {
    return m_recalculate_state_space;
}

template <typename OutputIndexEnum>
bool Bicycle<OutputIndexEnum>::need_recalculate_moore_parameters() const {
    return m_recalculate_moore_parameters;
}

/*
 * Ensure the normalization functions are updated if state indices change.
 *
 * We use 2*pi as the second argument simply to keep these angles from
 * growing toward infinity.
 * NOTE: This does not prevent the roll rate and steer rate from
 * growing to infinity.
 */
template <typename OutputIndexEnum>
typename Bicycle<OutputIndexEnum>::state_t Bicycle<OutputIndexEnum>::normalize_state(const state_t& x) const {
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::yaw_angle) == 0,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::roll_angle) == 1,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::steer_angle) == 2,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::roll_rate) == 3,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::steer_rate) == 4,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::number_of_types) == 5,
        "Invalid underlying value for state index element");

    state_t normalized_x = x;
    mod_two_pi_element<OutputIndexEnum>(normalized_x, state_index_t::yaw_angle);
    mod_two_pi_element<OutputIndexEnum>(normalized_x, state_index_t::roll_angle);
    mod_two_pi_element<OutputIndexEnum>(normalized_x, state_index_t::steer_angle);
    return normalized_x;
}

template <>
typename Bicycle<>::output_t Bicycle<>::normalize_output(const output_t& y) const {
    static_assert(index(Bicycle<>::output_index_t::yaw_angle) == 0,
        "Invalid underlying value for output index element");
    static_assert(index(Bicycle<>::output_index_t::steer_angle) == 1,
        "Invalid underlying value for output index element");
    static_assert(index(Bicycle<>::output_index_t::number_of_types) == 2,
        "Invalid underlying value for output index element");

    output_t normalized_y = y;

    auto yaw = std::fmod(get_element<output_index_t>(y, output_index_t::yaw_angle), constants::two_pi);
    if (yaw >= constants::pi) {
        yaw -= constants::two_pi;
    } else if (yaw < -constants::pi) {
        yaw += constants::two_pi;
    }

    set_element<output_index_t>(normalized_y, output_index_t::yaw_angle, yaw);
    mod_two_pi_element<output_index_t>(normalized_y, output_index_t::steer_angle);
    return normalized_y;
}
template <typename OutputIndexEnum>
typename Bicycle<OutputIndexEnum>::output_t Bicycle<OutputIndexEnum>::normalize_output(const output_t& y) const {
    return y;
}

template <typename OutputIndexEnum>
typename Bicycle<OutputIndexEnum>::auxiliary_state_t Bicycle<OutputIndexEnum>::normalize_auxiliary_state(const auxiliary_state_t& x_aux) const {
    static_assert(index(Bicycle<OutputIndexEnum>::auxiliary_state_index_t::x) == 0,
        "Invalid underlying value for auxiliary state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::auxiliary_state_index_t::y) == 1,
        "Invalid underlying value for auxiliary state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::auxiliary_state_index_t::rear_wheel_angle) == 2,
        "Invalid underlying value for auxiliary state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::auxiliary_state_index_t::pitch_angle) == 3,
        "Invalid underlying value for auxiliary state index element");
    static_assert(index(Bicycle<OutputIndexEnum>::auxiliary_state_index_t::number_of_types) == 4,
        "Invalid underlying value for auxiliary state index element");

    auxiliary_state_t normalized_x_aux = x_aux;
    mod_two_pi_element<OutputIndexEnum>(normalized_x_aux, auxiliary_state_index_t::rear_wheel_angle);
    mod_two_pi_element<OutputIndexEnum>(normalized_x_aux, auxiliary_state_index_t::pitch_angle);
    return normalized_x_aux;
}

template <typename OutputIndexEnum>
void Bicycle<OutputIndexEnum>::set_parameters_from_file(const char* param_file) {
    const unsigned int num_elem = o*o;
    std::array<real_t, 4*num_elem + 5> buffer;

    std::fstream pf(param_file, std::ios_base::in);
    if (!pf.good()) {
        throw std::invalid_argument("Invalid matrix parameter file provided.");
    }

    for (auto& d: buffer) {
        pf >> d;
    }
    pf.close();

    m_M = Eigen::Map<second_order_matrix_t>(buffer.data()).transpose();
    m_M_llt.compute(m_M);
    m_C1 = Eigen::Map<second_order_matrix_t>(buffer.data() + num_elem).transpose();
    m_K0 = Eigen::Map<second_order_matrix_t>(buffer.data() + 2*num_elem).transpose();
    m_K2 = Eigen::Map<second_order_matrix_t>(buffer.data() + 3*num_elem).transpose();
    m_w = buffer[4*num_elem];
    m_c = buffer[4*num_elem + 1];
    m_lambda = buffer[4*num_elem + 2];
    m_rr = buffer[4*num_elem + 3];
    m_rf = buffer[4*num_elem + 4];
}

template <typename OutputIndexEnum>
Bicycle<OutputIndexEnum>::Bicycle(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v, real_t dt):
    m_v(v), m_dt(dt),
    m_M(M), m_C1(C1), m_K0(K0), m_K2(K2),
    m_w(wheelbase), m_c(trail), m_lambda(steer_axis_tilt),
    m_rr(rear_wheel_radius), m_rf(front_wheel_radius),
    m_recalculate_state_space(true),
    m_recalculate_moore_parameters(true),
    m_M_llt(M),
    m_A(state_matrix_t::Zero()),
    m_B(input_matrix_t::Zero()),
    m_C(output_matrix_t::Zero()),
    m_D(feedthrough_matrix_t::Zero()) {
    set_moore_parameters();

    // This isn't called as it calls set_state_space(), a pure virtual function,
    // and the derived object is not yet constructed. We simply do the same thing
    // manually. Subsequent calls to set_v_dt() will work after object construction.
    // set_v_dt(v, dt);
    Bicycle<OutputIndexEnum>::set_state_space();

    // FIXME: call virtual function to set C and D that's only defined for
    // default OutputIndexEnum
    m_C(0, index(Bicycle<OutputIndexEnum>::output_index_t::yaw_angle)) = 1;
    m_C(1, index(Bicycle<OutputIndexEnum>::output_index_t::steer_angle)) = 1;
}

template <typename OutputIndexEnum>
Bicycle<OutputIndexEnum>::Bicycle(const char* param_file, real_t v, real_t dt) :
    m_v(v), m_dt(dt),
    m_recalculate_state_space(true),
    m_recalculate_moore_parameters(true),
    m_A(state_matrix_t::Zero()),
    m_B(input_matrix_t::Zero()),
    m_C(output_matrix_t::Zero()),
    m_D(feedthrough_matrix_t::Zero()) {
    // set M, C1, K0, K2 matrices and w, c, lambda, rr, rf parameters from file
    set_parameters_from_file(param_file);
    set_moore_parameters();

    // This isn't called as it calls set_state_space(), a pure virtual function,
    // and the derived object is not yet constructed. We simply do the same thing
    // manually. Subsequent calls to set_v_dt() will work after object construction.
    // set_v_dt(v, dt);
    Bicycle<OutputIndexEnum>::set_state_space();

    // FIXME: call virtual function to set C and D that's only defined for
    // default OutputIndexEnum
    m_C(0, index(Bicycle<OutputIndexEnum>::output_index_t::yaw_angle)) = 1;
    m_C(1, index(Bicycle<OutputIndexEnum>::output_index_t::steer_angle)) = 1;
}

#include "parameters.h"
template <typename OutputIndexEnum>
Bicycle<OutputIndexEnum>::Bicycle(real_t v, real_t dt) :
    Bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            parameters::benchmark::rear_wheel_radius,
            parameters::benchmark::front_wheel_radius,
            v, dt) {
/* Ensure state enum definitions are updated if state sizes change. */
static_assert(index(Bicycle<OutputIndexEnum>::input_index_t::number_of_types) == Bicycle<OutputIndexEnum>::m,
        "Invalid number of elements defined in input_index_t");
static_assert(index(Bicycle<OutputIndexEnum>::state_index_t::number_of_types) == Bicycle<OutputIndexEnum>::n,
        "Invalid number of elements defined in state_index_t");
static_assert(index(Bicycle<OutputIndexEnum>::output_index_t::number_of_types) == Bicycle<OutputIndexEnum>::l,
        "Invalid number of elements defined in output_index_t");
static_assert(index(Bicycle<OutputIndexEnum>::auxiliary_state_index_t::number_of_types) == Bicycle<OutputIndexEnum>::p,
        "Invalid number of elements defined in auxiliary_state_index_t");
static_assert(index(Bicycle<OutputIndexEnum>::full_state_index_t::number_of_types) == (Bicycle<OutputIndexEnum>::n + Bicycle<OutputIndexEnum>::p),
        "Invalid number of elements defined in full_state_index_t");
}

} // namespace model
