#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <tuple>
#include <Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>
#include <boost/math/tools/roots.hpp>
#include "bicycle.h"
#include "parameters.h"

namespace {
    const model::real_t discretization_precision = Eigen::NumTraits<model::real_t>::dummy_precision();

    template <typename E>
    constexpr uint8_t index(E e) {
        return static_cast<uint8_t>(e);
    }

} // namespace

namespace model {
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

Bicycle::Bicycle(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v, real_t dt, const state_space_map_t* discrete_state_space_map) :
    m_M(M), m_C1(C1), m_K0(K0), m_K2(K2),
    m_w(wheelbase), m_c(trail), m_lambda(steer_axis_tilt),
    m_rr(rear_wheel_radius), m_rf(front_wheel_radius),
    m_recalculate_state_space(true),
    m_recalculate_moore_parameters(true),
    m_C(parameters::defaultvalue::bicycle::C),
    m_D(parameters::defaultvalue::bicycle::D),
    m_discrete_state_space_map(discrete_state_space_map) {
    set_moore_parameters();

    // set forward speed, sampling time and update state matrices
    // state space matrices are set in set_v_dt() as m_recalculate_state_space is set to true
    set_v_dt(v, dt);
}

Bicycle::Bicycle(const char* param_file, real_t v, real_t dt,
        const state_space_map_t* discrete_state_space_map) :
    m_recalculate_state_space(true),
    m_recalculate_moore_parameters(true),
    m_C(parameters::defaultvalue::bicycle::C),
    m_D(parameters::defaultvalue::bicycle::D),
    m_discrete_state_space_map(discrete_state_space_map) {
    // set M, C1, K0, K2 matrices and w, c, lambda, rr, rf parameters from file
    set_parameters_from_file(param_file);
    set_moore_parameters();

    // set forward speed, sampling time and update state matrices
    // state space matrices are set in set_v_dt() as m_recalculate_state_space is set to true
    set_v_dt(v, dt);
}

Bicycle::Bicycle(real_t v, real_t dt,
        const state_space_map_t* discrete_state_space_map) :
    Bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            parameters::benchmark::rear_wheel_radius,
            parameters::benchmark::front_wheel_radius,
            v, dt, discrete_state_space_map) { }

bool Bicycle::auxiliary_state_field(full_state_index_t field) const {
    return index(field) < index(auxiliary_state_index_t::number_of_types);
}

Bicycle::state_t Bicycle::update_state(const Bicycle::state_t& x, const Bicycle::input_t& u, const Bicycle::output_t& z) const {
    (void)z;
    return update_state(x, u);
}

Bicycle::state_t Bicycle::update_state(const Bicycle::state_t& x, const Bicycle::input_t& u) const {
    return m_Ad*x + m_Bd*u;
}

Bicycle::output_t Bicycle::calculate_output(const Bicycle::state_t& x, const Bicycle::input_t& u) const {
    return m_C*x + m_D*u;
}

Bicycle::state_t Bicycle::update_state(const Bicycle::state_t& x) const {
    return m_Ad*x;
}

Bicycle::output_t Bicycle::calculate_output(const Bicycle::state_t& x) const {
    return m_C*x;
}

Bicycle::state_t Bicycle::integrate_state(const Bicycle::state_t& x, const Bicycle::input_t& u, real_t dt) const {
    odeint_state_t xout;

    xout << x, u;
    m_stepper.do_step([this](const odeint_state_t& xu, odeint_state_t& dxdt, const real_t t) -> void {
                (void)t;
                dxdt.head<n>() = m_A*xu.head<n>();
                // Normally we would write dxdt = A*x + B*u but B is not stored
                // explicitly as that would require the calculation of
                // M.inverse(). As B = [   0  ], the product Bu = [      0   ]
                //                     [ M^-1 ]                   [ M^-1 * u ]
                dxdt.segment<o>(n - o) += m_M_llt.solve(xu.segment<o>(n));
                dxdt.tail<m>().setZero();
            }, xout, 0.0, dt); // newly obtained state written in place
    return xout.head<n>();
}

Bicycle::state_t Bicycle::integrate_state(const Bicycle::state_t& x, real_t dt) const {
    state_t xout;

    m_stepper_noinput.do_step([this](const state_t& x, state_t& dxdt, const real_t t) -> void {
                (void)t;
                dxdt = m_A*x;
            }, x, 0.0, xout, dt);
    return xout;
}

Bicycle::auxiliary_state_t Bicycle::update_auxiliary_state(const state_t& x, const auxiliary_state_t& x_aux) const {
    odeint_auxiliary_state_t xout;

    xout << x_aux, x;
    m_auxiliary_stepper.do_step([this](
                const odeint_auxiliary_state_t& x, odeint_auxiliary_state_t& dxdt, const real_t t) -> void {
                (void)t;
                dxdt[index(full_state_index_t::x)] =
                    m_v*std::cos(x[index(full_state_index_t::yaw_angle)]); // xdot = v*cos(psi)
                dxdt[index(full_state_index_t::y)] =
                    m_v*std::sin(x[index(full_state_index_t::yaw_angle)]); // ydot = v*sin(psi)
                dxdt[index(full_state_index_t::rear_wheel_angle)] =
                    -m_v/m_rr;                                             // theta_rdot = -v/rr
                dxdt.tail<n + 1>().setZero(); // set state values + pitch angle to zero
            }, xout, 0.0, m_dt);

    // use last pitch angle as initial guess
    xout[index(full_state_index_t::pitch_angle)] =
        solve_constraint_pitch(x, x_aux[index(auxiliary_state_index_t::pitch_angle)]);
    return xout.head<p>();
}

void Bicycle::set_v_dt(real_t v, real_t dt) {
    if (m_recalculate_state_space) {
        initialize_state_space_matrices();
    }

    /* system state space is parameterized by forward speed v
     * this function sets forward speed and calculates the state space matrices
     * and additionally calculates discrete time state space if sampling time is nonzero
     */
    m_v = v;
    m_dt = dt;

    m_K = constants::g*m_K0 + v*v*m_K2;

    // M is positive definite so use the Cholesky decomposition in solving the linear system
    m_A(0, index(state_index_t::steer_angle)) = v * std::cos(m_lambda) / m_w; /* steer angle component of yaw rate */
    m_A.block<o, o>(3, 1) = -m_M_llt.solve(m_K);
    m_A.bottomRightCorner<o, o>() = -m_M_llt.solve(v*m_C1);

    // Calculate M^-1 as we need it for discretization
    if (o < 5) { // inverse calculation is okay if matrix size is small
        m_B.bottomRows<o>() = m_M.inverse();
    } else {
        m_B.bottomRows<o>() = m_M_llt.solve(second_order_matrix_t::Identity());
    }

    if (dt == 0.0) { // discrete time state does not change
        m_Ad.setIdentity();
        m_Bd.setZero();
    } else {
        state_space_map_key_t k = make_state_space_map_key(v, dt);
        if (!discrete_state_space_lookup(k)) {
            discretization_matrix_t AT = discretization_matrix_t::Zero();
            AT.topLeftCorner<n, n>() = m_A;
            AT.topRightCorner<n, m>() = m_B;
            AT *= dt;
            discretization_matrix_t T = AT.exp();
            if (!T.bottomLeftCorner<m, n>().isZero(discretization_precision) ||
                !T.bottomRightCorner<m, m>().isIdentity(discretization_precision)) {
                std::cout << "Warning: Discretization validation failed with v = " << v <<
                    ", dt = " << dt << ". Computation of Ad and Bd may be inaccurate.\n";
            }
            m_Ad = T.topLeftCorner<n, n>();
            m_Bd = T.topRightCorner<n, m>();
        }
    }
    m_recalculate_state_space = false;
}

bool Bicycle::discrete_state_space_lookup(const state_space_map_key_t& k) {
    if (m_discrete_state_space_map == nullptr) {
        return false;
    }
    auto search = m_discrete_state_space_map->find(k);
    if (search == m_discrete_state_space_map->end()) {
        return false;
    }

    // discrete state space matrices Ad, Bd have been provided for speed v, sample time dt.
    m_Ad = search->second.first;
    m_Bd = search->second.second;
    return true;
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

/*
 * Calculate constant parts of A, B from M, C1, K0, K2 matrices.
 */
void Bicycle::initialize_state_space_matrices() {
    m_A.setZero();
    m_B.setZero();
    m_Ad.setZero();
    m_Bd.setZero();

    // set constant parts of state and input matrices
    m_A(0, index(state_index_t::steer_rate)) = m_c * std::cos(m_lambda) / m_w; /* steer rate component of yaw rate */
    m_A.block<o, o>(1, 3).setIdentity();
    m_B.topRows<o>() = second_order_matrix_t::Zero();

    // We can write B in block matrix form as:
    // B.transpose() = [0 |  M.inverse().transpose()]
    // As M is positive definite, the Cholesky decomposition of M is stored and
    // used when necessary
    // m_B.bottomLeftCorner<o, o>() = m_M.inverse();
    m_M_llt.compute(m_M);
}

void Bicycle::set_state_space() {
    set_v_dt(m_v, m_dt);
}

/* set d1, d2, d3 used in pitch constraint calculation */
void Bicycle::set_moore_parameters() {
    m_d1 = std::cos(m_lambda)*(m_c + m_w - m_rr*std::tan(m_lambda));
    m_d3 = -std::cos(m_lambda)*(m_c - m_rf*std::tan(m_lambda));
    m_d2 = (m_rr + m_d1*std::sin(m_lambda) - m_rf + m_d3*std::sin(m_lambda)) / std::cos(m_lambda);
    m_recalculate_moore_parameters = false;
}

real_t Bicycle::solve_constraint_pitch(const state_t& x, real_t guess) const {
    // constraint function generated by script 'generate_pitch.py'.
    static constexpr int digits = std::numeric_limits<real_t>::digits*2/3;
    static constexpr real_t two = static_cast<real_t>(2.0);
    static constexpr real_t one_five = static_cast<real_t>(1.5);
    static const real_t min = -constants::pi/2;
    static const real_t max = constants::pi/2;
    auto constraint_function = [this, x](real_t pitch)->std::tuple<real_t, real_t> {
        return std::make_tuple(
((m_rf*std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two) +
(m_d3*std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) + std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)) +
m_rf*(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)])))*(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)])))*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)) +
std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) + std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*(-m_d1*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]),
two))*std::sin(pitch) + m_d2*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*std::cos(pitch) -
m_rr*std::cos(x[index(state_index_t::roll_angle)]))*std::cos(x[index(state_index_t::roll_angle)]))/(std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) + std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]),
two))*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)))
                ,
((m_rf*std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two) +
(m_d3*std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) + std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)) +
m_rf*(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)])))*(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)])))*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)) +
std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) + std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*(-m_d1*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]),
two))*std::sin(pitch) + m_d2*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*std::cos(pitch) -
m_rr*std::cos(x[index(state_index_t::roll_angle)]))*std::cos(x[index(state_index_t::roll_angle)]))*((-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]))*std::cos(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(pitch)*std::cos(pitch)*std::pow(std::cos(x[index(state_index_t::roll_angle)]),
two))/(std::pow(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) + std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two), one_five)*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))) +
((-m_d1*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*std::cos(pitch) - m_d2*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]),
two))*std::sin(pitch))*std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) + std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*std::cos(x[index(state_index_t::roll_angle)]) +
(-(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]))*std::cos(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) -
std::sin(pitch)*std::cos(pitch)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*(-m_d1*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]),
two))*std::sin(pitch) + m_d2*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*std::cos(pitch) -
m_rr*std::cos(x[index(state_index_t::roll_angle)]))*std::cos(x[index(state_index_t::roll_angle)])/std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) + std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)) +
(-two*m_rf*std::sin(pitch)*std::cos(pitch)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two) -
(m_d3*std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) + std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)) +
m_rf*(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)])))*std::cos(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
(m_d3*(-(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]))*std::cos(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) -
std::sin(pitch)*std::cos(pitch)*std::pow(std::cos(x[index(state_index_t::roll_angle)]),
two))/std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) + std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)) -
m_rf*std::cos(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]))*(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) +
std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)])))*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]),
two)))/(std::sqrt(std::pow(-std::sin(pitch)*std::cos(x[index(state_index_t::roll_angle)])*std::cos(x[index(state_index_t::steer_angle)]) + std::sin(x[index(state_index_t::roll_angle)])*std::sin(x[index(state_index_t::steer_angle)]), two) +
std::pow(std::cos(pitch), two)*std::pow(std::cos(x[index(state_index_t::roll_angle)]), two))*std::sqrt(std::pow(std::cos(x[index(state_index_t::roll_angle)]), two)))
                );
    };
    return boost::math::tools::newton_raphson_iterate(constraint_function, guess, min, max, digits);
}

real_t Bicycle::calculate_handlebar_feedback_torque(const state_t& x, const input_t& u) const {
    /*
     * The equations of motion for the Whipple model can be written as:
     *   M [phi_dd  ] + v*C1 [phi_d  ] + K [phi  ] = [T_phi  ]
     *     [delta_dd]        [delta_d]     [delta] = [T_delta]
     * where v*C1 is used to distinguish the "damping" matrix from the state
     * space output matrix C. In this simulation, T_phi defined to be 0 as there
     * is no way for the user nor the environment to supply a roll torque.
     *
     * The dynamics of the handlebar are governed by the following equation of
     * motion:
     *  I_delta * delta_dd = T_delta + T_m.
     *
     * Note: Positive torque and steer angle is clockwise as seen from the rider
     * looking down at the handlebars.
     *
     * In standard state space form, the equations of motion for the system are:
     *  [phi_d   ] = A [phi    ] + B [T_phi]
     *  [delta_d ]     [delta  ]     [T_delta]
     *  [phi_dd  ]     [phi_d  ]
     *  [delta_dd]     [delta_d]
     *
     * As we need an estimate of the steer angular acceleration, we use the
     * last row of A and B _with_ the assumption that the full state is available.
     * As the state vector is appended with yaw angle in this implementation,
     * this must be accounted for when calculating delta_dd.
     *
     * The output of this function is very susceptible to error/noise in the
     * state or noise in the input. For use with equipment, it is suggested to
     * filter the returned value.
     */
    auto steer_accel_index = index(state_index_t::steer_rate);
    real_t steer_acceleration = m_A.row(steer_accel_index)*x + m_B.row(steer_accel_index)*u;
    return steer_acceleration - u[index(input_index_t::steer_torque)];
}

} // namespace model
