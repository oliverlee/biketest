/*
 * Member function definitions of Oracle template class.
 * See oracle.h for template class declaration.
 */

namespace observer {

template <typename T>
Oracle<T>::Oracle(T& model) : Observer<T>(model, state_t::Zero()) {
    reset();
}

template <typename T>
Oracle<T>::Oracle(T& model, const state_t& x0) : Observer<T>(model, x0) { }

template <typename T>
void Oracle<T>::reset() {
    m_x.setZero();
}

template <typename T>
void Oracle<T>::update_state(const input_t& u, const measurement_t& z) {
    m_x = m_model.normalize_state(m_model.update_state(m_x, u, z));
}

} // namespace observer
