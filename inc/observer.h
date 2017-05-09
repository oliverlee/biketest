#pragma once
#include <Eigen/Core>
#include "types.h"
#include "discrete_linear.h"

namespace observer {
using real_t = model::real_t;

// This class cannot be instantiated and does not allow polymorphic deletion through a base pointer.
class ObserverBase {
    protected:
        ~ObserverBase() { }
};

template <typename Model>
class Observer : private ObserverBase {
    static_assert(std::is_base_of<model::DiscreteLinearBase, Model>::value, "Invalid template parameter type for Observer");

    public:
        using model_t = Model;
        using state_t = typename Model::state_t;
        using input_t = typename Model::input_t;
        using measurement_t = typename Model::output_t;

        virtual void reset() = 0;
        virtual void update_state(const input_t& u, const measurement_t& z) = 0;

        virtual void set_state(const state_t& x) { m_x = m_model.normalize_state(x); }

        virtual Model& model() const { return m_model; }
        virtual real_t dt() const { return m_model.dt(); }
        virtual const state_t& state() const { return m_x; }

    protected:
        Model& m_model;
        state_t m_x;

        Observer(Model& model, const state_t& x) : m_model(model), m_x(x) { }
};

} // namespace observer
