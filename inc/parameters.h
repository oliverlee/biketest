#pragma once
#include <Eigen/Dense>
#include "constants.h"
#include "bicycle/base.h"
#include "kalman.h"
#include "types.h"

namespace parameters {
    namespace benchmark {
        extern const model::BicycleBase::second_order_matrix_t M;
        extern const model::BicycleBase::second_order_matrix_t C1;
        extern const model::BicycleBase::second_order_matrix_t K0;
        extern const model::BicycleBase::second_order_matrix_t K2;
        extern const model::real_t wheelbase;
        extern const model::real_t trail;
        extern const model::real_t steer_axis_tilt;
        extern const model::real_t rear_wheel_radius;
        extern const model::real_t front_wheel_radius;
    } // namespace benchmark

    // default matrices used for examples and tests
    namespace defaultvalue {
        namespace kalman {
            // numbers given in degrees and then converted to radians
            extern const observer::Kalman<model::BicycleBase>::process_noise_covariance_t Q(model::real_t dt);
            extern const observer::Kalman<model::BicycleBase>::measurement_noise_covariance_t R;
        } // namespace kalman
    } // namespace defaultvalue
} // namespace parameters
