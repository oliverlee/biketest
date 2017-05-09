#pragma once
#include <Eigen/Core>
#include "types.h"

namespace model {

// This class cannot be instantiated and does not allow polymorphic deletion through a base pointer.
class DiscreteLinearBase {
    protected:
        ~DiscreteLinearBase() { }
};

template <size_t StateSize, size_t InputSize, size_t OutputSize, size_t SecondOrderStateSize>
class DiscreteLinear : private DiscreteLinearBase {
    public:
        static constexpr unsigned int n = StateSize; // state size
        static constexpr unsigned int m = InputSize; // input size
        static constexpr unsigned int l = OutputSize; // output size
        static constexpr unsigned int o = SecondOrderStateSize; // second order state size

        using state_t = Eigen::Matrix<real_t, n, 1>;
        using input_t = Eigen::Matrix<real_t, m, 1>;
        using output_t = Eigen::Matrix<real_t, l, 1>;
        using measurement_t = output_t;
        using state_matrix_t = Eigen::Matrix<real_t, n, n>;
        using input_matrix_t = Eigen::Matrix<real_t, n, m>;
        using output_matrix_t = Eigen::Matrix<real_t, l, n>;
        using feedthrough_matrix_t = Eigen::Matrix<real_t, l, m>;
        using second_order_matrix_t = Eigen::Matrix<real_t, o, o>;

        // The member function update_state(x, u, z) may not be any different than update_state(x, u)
        // depending on model implementation but must be defined for use with Oracle observer.
        virtual state_t update_state(const state_t& x, const input_t& u, const measurement_t& z) const = 0;
        virtual output_t calculate_output(const state_t& x, const input_t& u) const = 0;

        virtual const state_matrix_t& Ad() const = 0;
        virtual const input_matrix_t& Bd() const = 0;
        virtual const output_matrix_t& Cd() const = 0;
        virtual const feedthrough_matrix_t& Dd() const = 0;
        virtual real_t dt() const = 0;

        virtual state_t normalize_state(const state_t& x) const = 0;
        virtual output_t normalize_output(const output_t& y) const = 0;
};

} // namespace model
