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

namespace model {

BicycleWhipple::BicycleWhipple(const second_order_matrix_t& M, const second_order_matrix_t& C1,
        const second_order_matrix_t& K0, const second_order_matrix_t& K2,
        real_t wheelbase, real_t trail, real_t steer_axis_tilt,
        real_t rear_wheel_radius, real_t front_wheel_radius,
        real_t v) :
    Bicycle(M, C1, K0, K2, wheelbase, trail, steer_axis_tilt, rear_wheel_radius, front_wheel_radius, v) { }

BicycleWhipple::BicycleWhipple(const char* param_file, real_t v) : Bicycle(param_file, v) { }

BicycleWhipple::BicycleWhipple(real_t v) : Bicycle(v) { }

BicycleWhipple::state_t BicycleWhipple::integrate_state(real_t t, const BicycleWhipple::state_t& x, const BicycleWhipple::input_t& u, const BicycleWhipple::measurement_t& z) const {
    (void)z;
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
            }, xout, 0.0, t); // newly obtained state written in place
    return xout.head<n>();
}

} // namespace model
