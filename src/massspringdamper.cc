#include "massspringdamper.h"
#include <unsupported/Eigen/MatrixFunctions>

namespace {
#if !defined(NDEBUG)
        const real_t discretization_precision = Eigen::NumTraits<model::real_t>::dummy_precision();
#endif
} // namespace

MassSpringDamper::MassSpringDamper(real_t mass, real_t damping_constant,
                                   real_t spring_constant, real_t dt) :
m_mass(mass),
m_damping(damping_constant),
m_spring(spring_constant),
m_dt(dt) {
    set_discrete_state_space();
}

state_t update_state(const state_t& x, const input_t& u, const measurement_t& z = output_t::Zero()) const {
    (void)z;
    return m_A*x + m_B*u;
}

output_t calculate_output(const state_t& x, const input_t& u = input_t::Zero()) const {
    (void)u;
    return x;
}

const MassSpringDamper::state_matrix_t& MassSpringDamper::Ad() const {
    return m_Ad;
}

const MassSpringDamper::input_matrix_t& MassSpringDamper::Bd() const {
    return m_Bd;
}

const MassSpringDamper::output_matrix_t& MassSpringDamper::Cd() const {
    return output_matrix_t::Identity();
}

const MassSpringDamper::feedthrough_matrix_t& MassSpringDamper::Dd() const {
    return feedthrough_matrix_t::Zero();
}

real_t MassSpringDamper::dt() const {
    return m_dt;
}

void MassSpringDamper::set_dt(real_t dt) {
    m_dt = dt;
    set_discrete_state_space();
}

state_t MassSpringDamper::normalize_state(const state_t& x) const {
    return x;
}

state_t MassSpringDamper::normalize_output(const state_t& y) const {
    return y;
}

void MassSpringDamper::set_discrete_state_space() {
    // TODO: Both MassSpringDamper and Bicycle classes perform the same
    //       discretization procedure.
    m_Ad.setZero();
    m_Bd.setZero();

    if (m_dt == static_cast<real_t>(0)) { // discrete time state does not change
        m_Ad.setIdentity();
        m_Bd.setZero();
    } else {
        using discretization_matrix_t = Eigen::Matrix<real_t, n + m, n + m>;
        discretization_matrix_t AT = discretization_matrix_t::Zero();

        // m*x_ddot = -c*x_dot -k*x + F
        //
        // [ x_dot] = [    0    0]*[    x] + [  0]*F
        // [x_ddot]   [ -k/m -c/m] [x_dot]   [1/m]
        //
        AT(1, 0) = -m_spring/m_mass;
        AT(1, 1) = -m_damping/m_mass;
        AT(1, 2) = -1/m_mass;
        AT *= m_dt;

        discretization_matrix_t T = AT.exp();
        assert((T.bottomLeftCorner<m, n>().isZero(discretization_precision)) &&
               (T.bottomRightCorner<m, m>().isIdentity(discretization_precision)));
        m_Ad = T.topLeftCorner<n, n>();
        m_Bd = T.topRightCorner<n, m>();
    }
}
