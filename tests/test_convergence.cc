#include "test_convergence.h"
#include "parameters.h"

const model::real_t ConvergenceTest::m_yaw_tol = 1 * constants::as_radians;
const model::real_t ConvergenceTest::m_roll_tol = 0.3 * constants::as_radians;
const model::real_t ConvergenceTest::m_steer_tol = 0.3 * constants::as_radians;
const model::real_t ConvergenceTest::m_roll_rate_tol = 10 * ConvergenceTest::m_roll_tol;
const model::real_t ConvergenceTest::m_steer_rate_tol = 10 * ConvergenceTest::m_steer_tol;
const model::real_t ConvergenceTest::m_dt = 1.0/200;
const uint32_t ConvergenceTest::m_N = 1000;
const uint32_t ConvergenceTest::m_n = 100;

void ConvergenceTest::SetUp() {
    // set initial state
    m_x << 0, 3, 5, 0, 0; // define x in degrees
    m_x *= constants::as_radians;

    m_bicycle = new bicycle_t(GetParam(), m_dt);
    m_kalman = new kalman_t(*m_bicycle,
            bicycle_t::state_t::Zero(),
            parameters::defaultvalue::kalman::Q(m_dt),
            parameters::defaultvalue::kalman::R,
            std::pow(m_x[1]/2, 2) * bicycle_t::state_matrix_t::Identity());
    m_lqr = new lqr_t(*m_bicycle,
            lqr_t::state_cost_t::Identity(),
            0.1 * (lqr_t::input_cost_t() << 0, 0, 0, 1).finished(),
            bicycle_t::state_t::Zero(), m_n);

    m_gen = std::mt19937(m_rd());
    m_r0 = std::normal_distribution<>(0, parameters::defaultvalue::kalman::R(0, 0));
    m_r1 = std::normal_distribution<>(0, parameters::defaultvalue::kalman::R(1, 1));
    m_r1 = std::normal_distribution<>(0, parameters::defaultvalue::kalman::R(2, 2));
}

void ConvergenceTest::TearDown() {
    delete m_bicycle;
    delete m_kalman;
    delete m_lqr;
    m_bicycle = nullptr;
    m_kalman = nullptr;
    m_lqr = nullptr;
}

void ConvergenceTest::test_state_near(bicycle_t::state_t actual,
        bicycle_t::state_t expected, model::real_t tol_multiplier) {
    EXPECT_NEAR(actual(0), expected(0), tol_multiplier * m_yaw_tol);
    EXPECT_NEAR(actual(1), expected(1), tol_multiplier * m_roll_tol);
    EXPECT_NEAR(actual(2), expected(2), tol_multiplier * m_steer_tol);
    EXPECT_NEAR(actual(3), expected(3), tol_multiplier * m_roll_rate_tol);
    EXPECT_NEAR(actual(4), expected(4), tol_multiplier * m_steer_rate_tol);
}
