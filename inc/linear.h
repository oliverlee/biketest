#pragma once
#include <Eigen/Core>
#include "types.h"

namespace model {

/* This class cannot be instantiated and does not allow polymorphic deletion through a base pointer.*/
class LinearBase {
    protected:
        ~LinearBase() { }
};

template <size_t N, size_t M, size_t L, size_t O>
class Linear : private LinearBase {
    public:
        static constexpr unsigned int n = N; // state size
        static constexpr unsigned int m = M; // input size
        static constexpr unsigned int l = L; // output size
        static constexpr unsigned int o = O; // second order state size

        using state_t = Eigen::Matrix<real_t, n, 1>;
        using input_t = Eigen::Matrix<real_t, m, 1>;
        using output_t = Eigen::Matrix<real_t, l, 1>;
        using measurement_t = output_t;
        using state_matrix_t = Eigen::Matrix<real_t, n, n>;
        using input_matrix_t = Eigen::Matrix<real_t, n, m>;
        using output_matrix_t = Eigen::Matrix<real_t, l, n>;
        using feedthrough_matrix_t = Eigen::Matrix<real_t, l, m>;
        using second_order_matrix_t = Eigen::Matrix<real_t, o, o>;

        // The member function update_state(x, u, z) may not use measurement z
        // depending on model implementation but must be defined for use with Oracle observer.
        virtual state_t integrate_state(real_t t, const state_t& x, const input_t& u, const measurement_t& z) const = 0;
        virtual output_t calculate_output(const state_t& x, const input_t& u) const = 0;

        virtual const state_matrix_t& A() const = 0;
        virtual const input_matrix_t& B() const = 0;
        virtual const output_matrix_t& C() const = 0;
        virtual const feedthrough_matrix_t& D() const = 0;

        virtual state_t normalize_state(const state_t& x) const = 0;
        virtual output_t normalize_output(const output_t& y) const = 0;
};

} // namespace model
