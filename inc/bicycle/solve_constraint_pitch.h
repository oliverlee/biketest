#pragma once
#include "constants.h"

namespace model {

real_t solve_constraint_pitch(real_t roll, real_t steer, real_t guess, size_t max_iterations,
                              real_t m_d1, real_t m_d2, real_t m_d3, real_t m_rr, real_t m_rf);

} // namespace model
