#pragma once
#include "pole.h"
#include "observer.h"
#include <limits>

namespace observer {

/*
 * This template class implements a Luenberger observer.
 */
template <typename T>
class Luenberger final : public Observer<T> {
    public:
        /*
         * Although state_t, input_t, measurement_t are declared in the Observer<T> class
         * and are inherited, the type aliases are dependent names as the base class (Observer)
         * is a template dependent on the template parameter of Luenberger. In order to refer to
         * the type alias, we would need to specify 'typename Luenberger::state_t', thus it is
         * easier to simply redeclare the type aliases.
         */
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using measurement_t = typename T::output_t;
        using pole_vector_t = typename Eigen::Matrix<pole::pole_t, T::n, 1>;
        using luenberger_gain_t = typename Eigen::Matrix<real_t, T::n, T::l>;

        static constexpr real_t default_bandwidth = 1;

        Luenberger(T& system, const state_t& x0 = state_t::Zero(), real_t bandwidth = default_bandwidth);
        Luenberger(T& system, const state_t& x0, const pole_vector_t& pole_locations);
        Luenberger(T& system, const state_t& x0, const luenberger_gain_t& L);

        virtual void reset() override;
        virtual void update_state(const input_t& u, const measurement_t& z) override;

        void set_bandwidth(real_t bandwidth);
        void set_pole_locations(const pole_vector_t& pole_locations);
        void set_L(const luenberger_gain_t& L);

        real_t bandwidth() const;
        const pole_vector_t& pole_locations() const;
        const luenberger_gain_t& L() const;

    private:
        static constexpr real_t NaN = std::numeric_limits<real_t>::quiet_NaN();
        using Observer<T>::m_system;
        using Observer<T>::m_x;
        luenberger_gain_t m_L;
        pole_vector_t m_poles;
        real_t m_omega0;

        pole_vector_t calculate_pole_locations(real_t bandwidth) const;
        pole_vector_t calculate_pole_locations(const luenberger_gain_t& L) const;
        luenberger_gain_t calculate_luenberger_gain(const pole_vector_t& poles) const;
};  // class Luenberger

} // namespace observer

#include "luenberger.hh"
