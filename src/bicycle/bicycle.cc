#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <boost/math/tools/roots.hpp>
#include "bicycle/bicycle.h"
#include "parameters.h"

namespace {
    template <typename E>
    constexpr typename std::underlying_type<E>::type index(E e) {
        return static_cast<typename std::underlying_type<E>::type>(e);
    }
} // namespace

namespace model {
/* Ensure state enum definitions are updated if state sizes change. */
static_assert(index(Bicycle::input_index_t::number_of_types) == Bicycle::m,
        "Invalid number of elements defined in input_index_t");
static_assert(index(Bicycle::state_index_t::number_of_types) == Bicycle::n,
        "Invalid number of elements defined in state_index_t");
static_assert(index(Bicycle::output_index_t::number_of_types) == Bicycle::l,
        "Invalid number of elements defined in output_index_t");
static_assert(index(Bicycle::auxiliary_state_index_t::number_of_types) == Bicycle::p,
        "Invalid number of elements defined in auxiliary_state_index_t");
static_assert(index(Bicycle::full_state_index_t::number_of_types) == (Bicycle::n + Bicycle::p),
        "Invalid number of elements defined in full_state_index_t");

bool Bicycle::is_auxiliary_state_field(full_state_index_t field) {
    return index(field) < index(auxiliary_state_index_t::number_of_types);
}

Bicycle::output_t Bicycle::calculate_output(const state_t& x, const input_t& u) const {
    return m_C*x + m_D*u;
}

Bicycle::auxiliary_state_t Bicycle::integrate_auxiliary_state(const state_t& x, const auxiliary_state_t& x_aux, real_t t) const {
    full_state_t xout;
    // FIXME auxiliary state integration is incorrect. Refer to https://github.com/oliverlee/phobos/issues/63

    xout << x_aux, x;
    m_auxiliary_stepper.do_step([this](
                const full_state_t& x, full_state_t& dxdt, const real_t t) -> void {
                (void)t;
                dxdt[index(full_state_index_t::x)] =
                    m_v*std::cos(x[index(full_state_index_t::yaw_angle)]); // xdot = v*cos(psi)
                dxdt[index(full_state_index_t::y)] =
                    m_v*std::sin(x[index(full_state_index_t::yaw_angle)]); // ydot = v*sin(psi)
                dxdt[index(full_state_index_t::rear_wheel_angle)] =
                    -m_v/m_rr;                                             // theta_rdot = -v/rr
                dxdt.tail<n + 1>().setZero(); // set state values + pitch angle to zero
            }, xout, 0.0, t);

    // use last pitch angle as initial guess
    real_t roll = x[index(state_index_t::roll_angle)];
    real_t steer = x[index(state_index_t::steer_angle)];
    real_t pitch = x_aux[index(auxiliary_state_index_t::pitch_angle)];
    xout[index(full_state_index_t::pitch_angle)] =
        solve_constraint_pitch(roll, steer, pitch);
    return xout.head<p>();
}

void Bicycle::set_v(real_t v) {
    m_v = v;
    set_state_space();
}

void Bicycle::set_M(second_order_matrix_t& M, bool recalculate_state_space) {
    m_M = M;
    m_M_llt.compute(M);
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}

void Bicycle::set_C1(second_order_matrix_t& C1, bool recalculate_state_space) {
    m_C1 = C1;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}

void Bicycle::set_K0(second_order_matrix_t& K0, bool recalculate_state_space) {
    m_K0 = K0;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}

void Bicycle::set_K2(second_order_matrix_t& K2, bool recalculate_state_space) {
    m_K2 = K2;
    if (recalculate_state_space) {
        set_state_space();
    } else {
        m_recalculate_state_space = true;
    }
}

void Bicycle::set_wheelbase(real_t w, bool recalculate_parameters) {
    m_w = w;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}

void Bicycle::set_trail(real_t c, bool recalculate_parameters) {
    m_c = c;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}

void Bicycle::set_steer_axis_tilt(real_t lambda, bool recalculate_parameters) {
    m_lambda = lambda;
    if (recalculate_parameters) {
        set_moore_parameters();
        set_state_space();
    } else {
        m_recalculate_state_space = true;
        m_recalculate_moore_parameters = true;
    }
}

void Bicycle::set_rear_wheel_radius(real_t rr, bool recalculate_moore_parameters) {
    m_rr = rr;
    if (recalculate_moore_parameters) {
        set_moore_parameters();
    } else {
        m_recalculate_moore_parameters = true;
    }
}

void Bicycle::set_front_wheel_radius(real_t rf, bool recalculate_moore_parameters) {
    m_rf = rf;
    if (recalculate_moore_parameters) {
        set_moore_parameters();
    } else {
        m_recalculate_moore_parameters = true;
    }
}

void Bicycle::set_C(const output_matrix_t& C) {
    m_C = C;
}

void Bicycle::set_D(const feedthrough_matrix_t& D) {
    m_D = D;
}

void Bicycle::set_state_space() {
    static_assert(index(Bicycle::state_index_t::yaw_angle) == 0,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::roll_angle) == 1,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::steer_angle) == 2,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::roll_rate) == 3,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::steer_rate) == 4,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::number_of_types) == 5,
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
}

/* set d1, d2, d3 used in pitch constraint calculation */
void Bicycle::set_moore_parameters() {
    m_d1 = std::cos(m_lambda)*(m_c + m_w - m_rr*std::tan(m_lambda));
    m_d3 = -std::cos(m_lambda)*(m_c - m_rf*std::tan(m_lambda));
    m_d2 = (m_rr + m_d1*std::sin(m_lambda) - m_rf + m_d3*std::sin(m_lambda)) / std::cos(m_lambda);
    m_recalculate_moore_parameters = false;
}

real_t Bicycle::solve_constraint_pitch(real_t roll, real_t steer, real_t guess) const {
    // constraint function generated by script 'generate_pitch.py'.
    static constexpr int digits = std::numeric_limits<real_t>::digits*2/3;
    static constexpr real_t two = static_cast<real_t>(2.0);
    static constexpr real_t one_five = static_cast<real_t>(1.5);
    static constexpr real_t min = -constants::pi/2;
    static constexpr real_t max = constants::pi/2;
    auto constraint_function = [this, roll, steer](real_t pitch)->std::tuple<real_t, real_t> {
        return std::make_tuple(
((m_rf*std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two) +
(m_d3*std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two)) +
m_rf*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*std::sqrt(std::pow(std::cos(roll), two)) +
std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two))*(-m_d1*std::sqrt(std::pow(std::cos(roll),
two))*std::sin(pitch) + m_d2*std::sqrt(std::pow(std::cos(roll), two))*std::cos(pitch) -
m_rr*std::cos(roll))*std::cos(roll))/(std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer), two) + std::pow(std::cos(pitch), two)*std::pow(std::cos(roll),
two))*std::sqrt(std::pow(std::cos(roll), two)))
                ,
((m_rf*std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two) +
(m_d3*std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two)) +
m_rf*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*std::sqrt(std::pow(std::cos(roll), two)) +
std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two))*(-m_d1*std::sqrt(std::pow(std::cos(roll),
two))*std::sin(pitch) + m_d2*std::sqrt(std::pow(std::cos(roll), two))*std::cos(pitch) -
m_rr*std::cos(roll))*std::cos(roll))*((-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer))*std::cos(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(pitch)*std::cos(pitch)*std::pow(std::cos(roll),
two))/(std::pow(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two), one_five)*std::sqrt(std::pow(std::cos(roll), two))) +
((-m_d1*std::sqrt(std::pow(std::cos(roll), two))*std::cos(pitch) - m_d2*std::sqrt(std::pow(std::cos(roll),
two))*std::sin(pitch))*std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer), two) + std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two))*std::cos(roll) +
(-(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer))*std::cos(pitch)*std::cos(roll)*std::cos(steer) -
std::sin(pitch)*std::cos(pitch)*std::pow(std::cos(roll), two))*(-m_d1*std::sqrt(std::pow(std::cos(roll),
two))*std::sin(pitch) + m_d2*std::sqrt(std::pow(std::cos(roll), two))*std::cos(pitch) -
m_rr*std::cos(roll))*std::cos(roll)/std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer), two) + std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two)) +
(-two*m_rf*std::sin(pitch)*std::cos(pitch)*std::pow(std::cos(roll), two) -
(m_d3*std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two)) +
m_rf*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*std::cos(pitch)*std::cos(roll)*std::cos(steer) +
(m_d3*(-(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer))*std::cos(pitch)*std::cos(roll)*std::cos(steer) -
std::sin(pitch)*std::cos(pitch)*std::pow(std::cos(roll),
two))/std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two)) -
m_rf*std::cos(pitch)*std::cos(roll)*std::cos(steer))*(-std::sin(pitch)*std::cos(roll)*std::cos(steer) +
std::sin(roll)*std::sin(steer)))*std::sqrt(std::pow(std::cos(roll),
two)))/(std::sqrt(std::pow(-std::sin(pitch)*std::cos(roll)*std::cos(steer) + std::sin(roll)*std::sin(steer), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(roll), two))*std::sqrt(std::pow(std::cos(roll), two)))
                );
    };
    return boost::math::tools::newton_raphson_iterate(constraint_function, guess, min, max, digits);
}

const Bicycle::state_matrix_t& Bicycle::A() const {
    return m_A;
}

const Bicycle::input_matrix_t& Bicycle::B() const {
    return m_B;
}

const Bicycle::output_matrix_t& Bicycle::C() const {
    return m_C;
}

const Bicycle::feedthrough_matrix_t& Bicycle::D() const {
    return m_D;
}

const Bicycle::second_order_matrix_t& Bicycle::M() const {
    return m_M;
}

const Bicycle::second_order_matrix_t& Bicycle::C1() const {
    return m_C1;
}

const Bicycle::second_order_matrix_t& Bicycle::K0() const {
    return m_K0;
}

const Bicycle::second_order_matrix_t& Bicycle::K2() const {
    return m_K2;
}

real_t Bicycle::wheelbase() const {
    return m_w;
}

real_t Bicycle::trail() const {
    return m_c;
}

real_t Bicycle::steer_axis_tilt() const {
    return m_lambda;
}

real_t Bicycle::rear_wheel_radius() const {
    return m_rr;
}

real_t Bicycle::front_wheel_radius() const {
    return m_rf;
}

real_t Bicycle::v() const {
    return m_v;
}

bool Bicycle::need_recalculate_state_space() const {
    return m_recalculate_state_space;
}

bool Bicycle::need_recalculate_moore_parameters() const {
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
Bicycle::state_t Bicycle::normalize_state(const state_t& x) const {
    static_assert(index(Bicycle::state_index_t::yaw_angle) == 0,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::roll_angle) == 1,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::steer_angle) == 2,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::roll_rate) == 3,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::steer_rate) == 4,
        "Invalid underlying value for state index element");
    static_assert(index(Bicycle::state_index_t::number_of_types) == 5,
        "Invalid underlying value for state index element");

    constexpr auto yaw_index = index(Bicycle::state_index_t::yaw_angle);
    constexpr auto roll_index = index(Bicycle::state_index_t::roll_angle);
    constexpr auto steer_index = index(Bicycle::state_index_t::steer_angle);

    state_t normalized_x = x;
    normalized_x[yaw_index] = std::fmod(x[yaw_index], constants::two_pi);
    normalized_x[roll_index] = std::fmod(x[roll_index], constants::two_pi);
    normalized_x[steer_index] = std::fmod(x[steer_index], constants::two_pi);
    return normalized_x;
}

Bicycle::output_t Bicycle::normalize_output(const output_t& y) const {
    static_assert(index(Bicycle::output_index_t::yaw_angle) == 0,
        "Invalid underlying value for output index element");
    static_assert(index(Bicycle::output_index_t::steer_angle) == 1,
        "Invalid underlying value for output index element");
    static_assert(index(Bicycle::output_index_t::number_of_types) == 2,
        "Invalid underlying value for output index element");

    constexpr auto yaw_index = index(Bicycle::output_index_t::yaw_angle);
    constexpr auto steer_index = index(Bicycle::output_index_t::steer_angle);

    output_t normalized_y = y;
    normalized_y[yaw_index] = std::fmod(y[yaw_index], constants::two_pi);
    normalized_y[steer_index] = std::fmod(y[steer_index], constants::two_pi);
    return normalized_y;
}

Bicycle::auxiliary_state_t Bicycle::normalize_auxiliary_state(const auxiliary_state_t& x_aux) const {
    static_assert(index(Bicycle::auxiliary_state_index_t::x) == 0,
        "Invalid underlying value for auxiliary state index element");
    static_assert(index(Bicycle::auxiliary_state_index_t::y) == 1,
        "Invalid underlying value for auxiliary state index element");
    static_assert(index(Bicycle::auxiliary_state_index_t::rear_wheel_angle) == 2,
        "Invalid underlying value for auxiliary state index element");
    static_assert(index(Bicycle::auxiliary_state_index_t::pitch_angle) == 3,
        "Invalid underlying value for auxiliary state index element");
    static_assert(index(Bicycle::auxiliary_state_index_t::number_of_types) == 4,
        "Invalid underlying value for auxiliary state index element");

    constexpr auto rear_wheel_index = index(Bicycle::auxiliary_state_index_t::rear_wheel_angle);
    constexpr auto pitch_index = index(Bicycle::auxiliary_state_index_t::pitch_angle);

    auxiliary_state_t normalized_x_aux = x_aux;
    normalized_x_aux[rear_wheel_index] = std::fmod(x_aux[rear_wheel_index], constants::two_pi);
    normalized_x_aux[pitch_index] = std::fmod(x_aux[pitch_index], constants::two_pi);
    return normalized_x_aux;
}

void Bicycle::set_parameters_from_file(const char* param_file) {
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
    m_C1 = Eigen::Map<second_order_matrix_t>(buffer.data() + num_elem).transpose();
    m_K0 = Eigen::Map<second_order_matrix_t>(buffer.data() + 2*num_elem).transpose();
    m_K2 = Eigen::Map<second_order_matrix_t>(buffer.data() + 3*num_elem).transpose();
    m_w = buffer[4*num_elem];
    m_c = buffer[4*num_elem + 1];
    m_lambda = buffer[4*num_elem + 2];
    m_rr = buffer[4*num_elem + 3];
    m_rf = buffer[4*num_elem + 4];
}

Bicycle::Bicycle(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v):
    m_M(M), m_C1(C1), m_K0(K0), m_K2(K2),
    m_w(wheelbase), m_c(trail), m_lambda(steer_axis_tilt),
    m_rr(rear_wheel_radius), m_rf(front_wheel_radius),
    m_recalculate_state_space(true),
    m_recalculate_moore_parameters(true),
    m_A(state_matrix_t::Zero()),
    m_B(input_matrix_t::Zero()),
    m_C(parameters::defaultvalue::bicycle::C),
    m_D(parameters::defaultvalue::bicycle::D) {
    set_moore_parameters();
    set_v(v);
}

Bicycle::Bicycle(const char* param_file, real_t v):
    m_recalculate_state_space(true),
    m_recalculate_moore_parameters(true),
    m_A(state_matrix_t::Zero()),
    m_B(input_matrix_t::Zero()),
    m_C(parameters::defaultvalue::bicycle::C),
    m_D(parameters::defaultvalue::bicycle::D) {
    // set M, C1, K0, K2 matrices and w, c, lambda, rr, rf parameters from file
    set_parameters_from_file(param_file);
    set_moore_parameters();
    set_v(v);
}

Bicycle::Bicycle(real_t v):
    Bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            parameters::benchmark::rear_wheel_radius,
            parameters::benchmark::front_wheel_radius,
            v) { }

} // namespace model
