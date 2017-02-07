#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <tuple>
#include <unsupported/Eigen/MatrixFunctions>
#include <boost/math/tools/roots.hpp>
#include "bicycle/whipple.h"
#include "parameters.h"

namespace {
    const model::real_t discretization_precision = Eigen::NumTraits<model::real_t>::dummy_precision();

    template <typename E>
    constexpr uint8_t index(E e) {
        return static_cast<uint8_t>(e);
    }

} // namespace

namespace model {
BicycleWhipple::BicycleWhipple(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
        , const state_space_map_t* discrete_state_space_map
#endif // BICYCLE_USE_STATE_SPACE_MAP
        ) :
    Bicycle(M, C1, K0, K2, wheelbase, trail, steer_axis_tilt, rear_wheel_radius, front_wheel_radius, v, dt)
#if BICYCLE_USE_STATE_SPACE_MAP
    , m_discrete_state_space_map(discrete_state_space_map)
#endif // BICYCLE_USE_STATE_SPACE_MAP
    {
    set_discrete_state_space();
}

BicycleWhipple::BicycleWhipple(const char* param_file, real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
        , const state_space_map_t* discrete_state_space_map
#endif // BICYCLE_USE_STATE_SPACE_MAP
        ) :
    Bicycle(param_file, v, dt)
#if BICYCLE_USE_STATE_SPACE_MAP
    , m_discrete_state_space_map(discrete_state_space_map)
#endif // BICYCLE_USE_STATE_SPACE_MAP
    {
    set_discrete_state_space();
}

BicycleWhipple::BicycleWhipple(real_t v, real_t dt
#if BICYCLE_USE_STATE_SPACE_MAP
        , const state_space_map_t* discrete_state_space_map
#endif // BICYCLE_USE_STATE_SPACE_MAP
        ) :
    Bicycle(v, dt)
#if BICYCLE_USE_STATE_SPACE_MAP
    , m_discrete_state_space_map(discrete_state_space_map)
#endif // BICYCLE_USE_STATE_SPACE_MAP
    {
    set_discrete_state_space();
}

BicycleWhipple::state_t BicycleWhipple::update_state(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u, const BicycleWhipple::measurement_t& z) const {
    (void)z;
    return m_Ad*x + m_Bd*u;
}

BicycleWhipple::output_t BicycleWhipple::calculate_output(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u) const {
    return m_C*x + m_D*u;
}

BicycleWhipple::full_state_t BicycleWhipple::integrate_full_state(const BicycleWhipple::full_state_t& xf, const BicycleWhipple::input_t& u, real_t t) const {
    static constexpr auto x_index = index(full_state_index_t::x);
    static constexpr auto y_index = index(full_state_index_t::y);
    static constexpr auto rear_wheel_index = index(full_state_index_t::rear_wheel_angle);
    static constexpr auto pitch_index = index(full_state_index_t::pitch_angle);
    static constexpr auto yaw_index = index(full_state_index_t::yaw_angle);

    const real_t v = m_v;
    const real_t rr = m_rr;
    const state_matrix_t& A = m_A;
    const Eigen::LLT<second_order_matrix_t>& M_llt = m_M_llt;

    full_state_t xout = xf;

    m_stepper.do_step([&A, &M_llt, &u, v, rr](const full_state_t& x, full_state_t& dxdt, const real_t t) -> void {
            (void)t; // system is time-independent

            // auxiliary state fields first
            dxdt[x_index] = v*std::cos(x[yaw_index]);
            dxdt[y_index] = v*std::sin(x[yaw_index]);
            dxdt[rear_wheel_index] = -v/rr;
            dxdt[pitch_index] = 0; // pitch angle is not integrated and must be obtained using solve_pitch_constraint()

            // state fields
            dxdt.tail<n>() = A*x.tail<n>();
            // Normally we would write dxdt = A*x + B*u but we prefer not to
            // use the matrix inverse unless absolutely necessary when computing.
            // As B = [   0  ], the product Bu = [      0   ]
            //        [ M^-1 ]                   [ M^-1 * u ]
            dxdt.tail<o>() += M_llt.solve(u);
            }, xout, static_cast<real_t>(0), t); // newly obtained state written in place
    return xout;
}

BicycleWhipple::state_t BicycleWhipple::integrate_state(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u, real_t t) const {
    state_t xout = x;

    const state_matrix_t& A = m_A;
    const Eigen::LLT<second_order_matrix_t>& M_llt = m_M_llt;

    m_stepper_state.do_step([&A, &M_llt, &u](const state_t& x, state_t& dxdt, const real_t t) -> void {
            (void)t; // system is time-independent
            dxdt = A*x;
            dxdt.tail<o>() += M_llt.solve(u);
            }, xout, static_cast<real_t>(0), t); // newly obtained state written in place
    return xout;
}

void BicycleWhipple::set_state_space() {
    /*
     * System state space is parameterized by forward speed v so this function
     * is always called after setting v. This function calculates the state
     * space matrices and additionally calculates discrete time state space if
     * sampling time is nonzero.
     */
    Bicycle::set_state_space();
    set_discrete_state_space();
}

#if BICYCLE_USE_STATE_SPACE_MAP
bool BicycleWhipple::discrete_state_space_lookup(real_t v, real_t dt) {
    if (m_discrete_state_space_map == nullptr) {
        return false;
    }

    state_space_map_key_t k = make_state_space_map_key(v, dt);
    auto search = m_discrete_state_space_map->find(k);
    if (search == m_discrete_state_space_map->end()) {
        return false;
    }

    // discrete state space matrices Ad, Bd have been provided for speed v, sample time dt.
    m_Ad = search->second.first;
    m_Bd = search->second.second;
    return true;
}
#endif // BICYCLE_USE_STATE_SPACE_MAP

const BicycleWhipple::state_matrix_t& BicycleWhipple::Ad() const {
    return m_Ad;
}

const BicycleWhipple::input_matrix_t& BicycleWhipple::Bd() const {
    return m_Bd;
}

const BicycleWhipple::output_matrix_t& BicycleWhipple::Cd() const {
    return C();
}

const BicycleWhipple::feedthrough_matrix_t& BicycleWhipple::Dd() const {
    return D();
}

void BicycleWhipple::set_discrete_state_space() {
    m_Ad.setZero();
    m_Bd.setZero();

    if (m_dt == 0.0) { // discrete time state does not change
        m_Ad.setIdentity();
        m_Bd.setZero();
    } else {
#if BICYCLE_USE_STATE_SPACE_MAP
        if (!discrete_state_space_lookup(m_v, m_dt)) {
#endif // BICYCLE_USE_STATE_SPACE_MAP
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
            if (!T.bottomLeftCorner<m, n>().isZero(discretization_precision) ||
                !T.bottomRightCorner<m, m>().isIdentity(discretization_precision)) {
                std::cout << "Warning: Discretization validation failed with v = " << m_v <<
                    ", dt = " << m_dt << ". Computation of Ad and Bd may be inaccurate.\n";
            }
            m_Ad = T.topLeftCorner<n, n>();
            m_Bd = T.topRightCorner<n, m>();
#if BICYCLE_USE_STATE_SPACE_MAP
        }
#endif // BICYCLE_USE_STATE_SPACE_MAP
    }
}

} // namespace model
