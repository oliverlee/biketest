#include "gtest/gtest.h"
#include "test_convergence.h"

// TODO: Add more extensive testing
class LqrKalmanConvergenceTest: public ConvergenceTest {
    public:
        void simulate() {
            for(unsigned int i = 0; i < m_N; ++i) {
                auto u = m_lqr->control_calculate(m_kalman->x());
                m_x = m_bicycle->update_state(m_x, u);

                auto z = m_bicycle->calculate_output(m_x);
                z(0) += m_r0(m_gen);
                z(1) += m_r1(m_gen);
                z(2) += m_r1(m_gen);

                m_kalman->time_update(u);
                m_kalman->measurement_update(z);
            }
        }
};

TEST_P(LqrKalmanConvergenceTest, ZeroReference) {
    simulate();
    test_state_near(m_kalman->x(), x_true());
}

INSTANTIATE_TEST_CASE_P(
    ConvergenceRange_1_9,
    LqrKalmanConvergenceTest,
    ::testing::Range(static_cast<model::real_t>(0.5),
        static_cast<model::real_t>(9.5),
        static_cast<model::real_t>(0.5)));
