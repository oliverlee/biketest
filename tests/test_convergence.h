#pragma once
#include <random>
#include "gtest/gtest.h"
#include "bicycle/whipple.h"
#include "kalman.h"
#include "lqr.h"
#include "types.h"

class ConvergenceTest: public ::testing::TestWithParam<model::real_t> {
    public:
        using bicycle_t = model::BicycleWhipple;
        using kalman_t = observer::Kalman<bicycle_t>;
        using lqr_t = controller::Lqr<bicycle_t>;
        virtual void SetUp();
        virtual void TearDown();
        virtual void simulate() = 0;
        void test_state_near(model::BicycleWhipple::state_t actual,
                model::BicycleWhipple::state_t expected, model::real_t tol_multiplier = 1.0);
        model::BicycleWhipple::state_t x_true();

    protected:
        static const model::real_t m_yaw_tol;
        static const model::real_t m_roll_tol;
        static const model::real_t m_steer_tol;
        static const model::real_t m_roll_rate_tol;
        static const model::real_t m_steer_rate_tol;
        static const model::real_t m_dt;
        static const uint32_t m_N; // simulation length
        static const uint32_t m_n; // horizon length
        model::BicycleWhipple* m_bicycle;
        observer::Kalman<model::BicycleWhipple>* m_kalman;
        controller::Lqr<model::BicycleWhipple>* m_lqr;
        model::BicycleWhipple::state_t m_x;

        std::random_device m_rd; // used only to seed rng
        std::mt19937 m_gen;
        std::normal_distribution<> m_r0;
        std::normal_distribution<> m_r1;
        std::normal_distribution<> m_r2;
};

inline model::BicycleWhipple::state_t ConvergenceTest::x_true() {
    return m_x;
}
