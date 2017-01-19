#pragma once
#include <Eigen/Core>
#include "types.h"
#include "discrete_linear.h"

namespace observer {
using real_t = model::real_t;

/* This class cannot be instantiated and does not allow polymorphic deletion through a base pointer.*/
class ObserverBase {
    protected:
        ~ObserverBase() { }
};

template <typename T>
class Observer : private ObserverBase {
    static_assert(
            (std::is_base_of<model::LinearBase, T>::value || std::is_base_of<model::DiscreteLinearBase, T>::value),
            "Invalid template parameter type for Observer");

    public:
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using measurement_t = typename T::output_t;

        virtual void reset() = 0;
        virtual void update_state(const input_t& u, const measurement_t& z) = 0;

        virtual void set_state(const state_t& x) { m_x = m_system.normalize_state(x); }

        virtual T& system() const { return m_system; }
        virtual real_t dt() const { return m_system.dt(); }
        virtual const state_t& state() const { return m_x; }

    protected:
        T& m_system;
        state_t m_x;

        Observer(T& system, const state_t& x) : m_system(system), m_x(x) { }
};

} // namespace observer
