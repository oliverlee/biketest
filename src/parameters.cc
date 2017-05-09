#include <cmath>
#include "parameters.h"

namespace parameters {

namespace benchmark {
    const model::Bicycle::second_order_matrix_t M(
            (model::Bicycle::second_order_matrix_t() <<
             80.81722, 2.31941332208709, 2.31941332208709, 0.29784188199686).finished());
    const model::Bicycle::second_order_matrix_t C1(
            (model::Bicycle::second_order_matrix_t() <<
             0.0, 33.86641391492494, -0.85035641456978, 1.6854039739756).finished());
    const model::Bicycle::second_order_matrix_t K0(
            (model::Bicycle::second_order_matrix_t() <<
             -80.95, -2.59951685249872, -2.59951685249872, -0.80329488458618).finished());
    const model::Bicycle::second_order_matrix_t K2(
            (model::Bicycle::second_order_matrix_t() <<
             0.0, 76.59734589573222, 0.0, 2.65431523794604).finished());
    const model::real_t wheelbase = 1.02;
    const model::real_t trail = 0.08;
    const model::real_t steer_axis_tilt = boost::math::constants::pi<model::real_t>()/10.0;
    const model::real_t rear_wheel_radius = 0.3;
    const model::real_t front_wheel_radius = 0.35;
} // namespace benchmark

namespace defaultvalue {

namespace bicycle {
    const model::Bicycle::output_matrix_t C(
            (model::Bicycle::output_matrix_t() <<
             1, 0, 0, 0, 0,                // yaw angle
             0, 0, 1, 0, 0,                // steer angle
             0, 0, 0, 0, 1).finished());   // steer rate
    const model::Bicycle::feedthrough_matrix_t D(model::Bicycle::feedthrough_matrix_t::Zero());
} // namespace bicycle

namespace kalman {
   // numbers given in degrees and then converted to radians
   const observer::Kalman<model::Bicycle>::process_noise_covariance_t Q(model::real_t dt) {
       /*
        * Q form of constant-velocity particle. From discrete-time Kalman filter
        * notes: Estimation II. Ian Reid. Hilary Term, 2001
        * http://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf
        */
       const model::real_t data[] = {dt,       0,       0,          0,          0,
                                      0,      dt,       0,    dt*dt/2,          0,
                                      0,       0,      dt,          0,    dt*dt/2,
                                      0, dt*dt/2,       0, dt*dt*dt/3,          0,
                                      0,       0, dt*dt/2,          0, dt*dt*dt/3};
       return std::pow(0.1*constants::as_radians, 2) *
           observer::Kalman<model::Bicycle>::process_noise_covariance_t(data);
   }
   const observer::Kalman<model::Bicycle>::measurement_noise_covariance_t R(
           (observer::Kalman<model::Bicycle>::measurement_noise_covariance_t() <<
               0.1,     0,     0,
                 0, 0.008,     0,
                 0,     0, 0.012).finished() * constants::as_radians);
} // namesapce kalman

} // namespace defaultvalue

} // namespace parameters
