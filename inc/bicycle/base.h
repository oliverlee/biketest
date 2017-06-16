#pragma once
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "discrete_linear.h"

namespace model {

class BicycleBase : public DiscreteLinear <5, 2, 2, 2> {
    // TODO: Move bicycle code independent of output size here
};

} // namespace model;
