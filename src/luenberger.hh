#include "constants.h"
#include "exception.h"
#include <Eigen/Eigenvalues>
#include <cassert>
/*
 * Member function definitions of Luenberger template class.
 * See luenberger.h for template class declaration.
 */

namespace observer {

template <typename T>
Luenberger<T>::Luenberger(T& system, const state_t& x0, real_t bandwidth) :
        Observer<T>(system, x0),
        m_L(luenberger_gain_t::Zero()),
        m_poles(pole_vector_t::Zero()),
        m_omega0(NaN) {
    set_bandwidth(bandwidth);
}

template <typename T>
Luenberger<T>::Luenberger(T& system, const state_t& x0, const pole_vector_t& pole_locations) :
        Observer<T>(system, x0),
        m_L(luenberger_gain_t::Zero()),
        m_poles(pole_vector_t::Zero()),
        m_omega0(NaN) {
    set_pole_locations(pole_locations);
}

template <typename T>
Luenberger<T>::Luenberger(T& system, const state_t& x0, const luenberger_gain_t& L) :
        Observer<T>(system, x0),
        m_L(luenberger_gain_t::Zero()),
        m_poles(pole_vector_t::Zero()),
        m_omega0(NaN) {
    set_L(L);
}

template <typename T>
void Luenberger<T>::reset() {
    m_x.setZero();
}

template <typename T>
void Luenberger<T>::update_state(const input_t& u, const measurement_t& z) {
    const measurement_t y = m_system.Cd()*m_x + m_system.Dd()*u;
    m_x = m_system.normalize_state(m_system.Ad()*m_x + m_system.Bd()*u + m_L*(y - z));
}

/*
 * This takes a bandwidth and calculates the pole location of the
 * _continuous_time_ system, placing them in a Butterworth configuration.
 * As all observers operate on a discrete time linear system, the poles are
 * converted to discrete time before the Luenberger feedback gain is calculated.
 */
template <typename T>
void Luenberger<T>::set_bandwidth(real_t bandwidth) {
    m_omega0 = bandwidth;
    m_poles = calculate_pole_locations(bandwidth);
    m_L = calculate_luenberger_gain(m_poles);
}

template <typename T>
void Luenberger<T>::set_pole_locations(const pole_vector_t& pole_locations) {
    m_omega0 = NaN;
    m_poles = pole_locations;
    m_L = calculate_luenberger_gain(pole_locations);
}

template <typename T>
void Luenberger<T>::set_L(const luenberger_gain_t& L) {
    m_omega0 = NaN;
    m_poles = calculate_pole_locations(L);
    m_L = L;
}

template <typename T>
inline real_t Luenberger<T>::bandwidth() const {
    return m_omega0;
}

template <typename T>
inline const typename Luenberger<T>::pole_vector_t& Luenberger<T>::pole_locations() const {
    return m_poles;
}

template <typename T>
inline const typename Luenberger<T>::luenberger_gain_t& Luenberger<T>::L() const {
    return m_L;
}

template <typename T>
typename Luenberger<T>::pole_vector_t Luenberger<T>::calculate_pole_locations(real_t bandwidth) const {
    /*
     * poles are given by the roots of:
     *   (s/omega0)^(2k) = (-1)^(k + 1)
     *
     * where k is the number of poles in the left half-plane
     * (and k = T::n for our class)
     * and are said to have a "Butterworth configuration"
     */
    static constexpr auto i = pole::pole_t{static_cast<real_t>(0), static_cast<real_t>(1)};
    static constexpr real_t pole_angular_distance = constants::pi/T::n;

    pole_vector_t poles;
    real_t angle = (pole_angular_distance + constants::pi)/2;
    for (unsigned int j = 0; j < T::n; ++j) {
        poles[j] = bandwidth * std::exp(i*angle);
        angle += pole_angular_distance;
    }
    return pole::continuous_to_discrete(poles, m_system.dt());
}

template <typename T>
typename Luenberger<T>::pole_vector_t Luenberger<T>::calculate_pole_locations(const luenberger_gain_t& L) const {
    Eigen::EigenSolver<typename T::state_matrix_t> es;
    es.compute(m_system.Ad() - L*m_system.Cd(), false); /* compute only eigenvalues */
    if (es.info() != Eigen::ComputationInfo::Success) {
        assert(false); // TODO: handle computation failure
    }
    m_poles = es.eigenvalues();
}

template <typename T>
typename Luenberger<T>::luenberger_gain_t Luenberger<T>::calculate_luenberger_gain(const pole_vector_t& poles) const {
    return pole::place(m_system.Ad().transpose(), m_system.Cd().transpose(), poles).transpose();
}

} // namespace observer
