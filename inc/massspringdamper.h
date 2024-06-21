#pragma once
#include "discrete_linear.h"

namespace model {

/* State, Input, and Output Definitions
 * state: [position, velocity]
 * input: [force (or torque)]
 * output: [position, velocity]
 *
 * This model represents a simple translational or rotational mass with
 * spring and damper and full state output.
 */

class MassSpringDamper final : public DiscreteLinear<2, 1, 2, 0> {
    public:
        MassSpringDamper(real_t mass, real_t damping_constant, real_t spring_constant, real_t dt);

        virtual state_t update_state(const state_t& x, const input_t& u, const measurement_t& z = output_t::Zero()) const override;
        virtual output_t calculate_output(const state_t& x, const input_t& u = input_t::Zero()) const override;

        virtual const state_matrix_t& Ad() const override;
        virtual const input_matrix_t& Bd() const override;
        virtual const output_matrix_t& Cd() const override;
        virtual const feedthrough_matrix_t& Dd() const override;
        virtual real_t dt() const override;
        void set_dt(real_t t);

        virtual state_t normalize_state(const state_t& x) const override;
        virtual output_t normalize_output(const output_t& y) const override;

    private:
        real_t m_mass;
        real_t m_damping:
        real_t m_spring;
        state_matrix_t m_Ad;
        state_matrix_t m_Bd;

        void set_discrete_state_space();
};

} //namespace model
