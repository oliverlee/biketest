#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <tuple>
#include <Eigen/QR>
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
        real_t v, real_t dt, const state_space_map_t* discrete_state_space_map) :
    Bicycle(M, C1, K0, K2, wheelbase, trail, steer_axis_tilt, rear_wheel_radius, front_wheel_radius, v, dt),
    m_discrete_state_space_map(discrete_state_space_map) { }

BicycleWhipple::BicycleWhipple(const char* param_file, real_t v, real_t dt,
        const state_space_map_t* discrete_state_space_map) :
    Bicycle(param_file, v, dt),
    m_discrete_state_space_map(discrete_state_space_map) { }

BicycleWhipple::BicycleWhipple(real_t v, real_t dt,
        const state_space_map_t* discrete_state_space_map) :
    Bicycle(v, dt),
    m_discrete_state_space_map(discrete_state_space_map) { }

BicycleWhipple::state_t BicycleWhipple::update_state(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u, const BicycleWhipple::measurement_t& z) const {
    (void)z;
    return m_Ad*x + m_Bd*u;
}

BicycleWhipple::output_t BicycleWhipple::calculate_output(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u) const {
    return m_C*x + m_D*u;
}

BicycleWhipple::state_t BicycleWhipple::integrate_state(const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u, real_t dt) const {
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

void BicycleWhipple::set_state_space() {
    /*
     * System state space is parameterized by forward speed v so this function
     * is always called after setting v. This function calculates the state
     * space matrices and additionally calculates discrete time state space if
     * sampling time is nonzero.
     */

    Bicycle::set_state_space();

    m_Ad.setZero();
    m_Bd.setZero();

    if (m_dt == 0.0) { // discrete time state does not change
        m_Ad.setIdentity();
        m_Bd.setZero();
    } else {
        if (!discrete_state_space_lookup(m_v, m_dt)) {
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
        }
    }
}

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

} // namespace model
