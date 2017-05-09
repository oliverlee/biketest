#include <Eigen/Cholesky>
/*
 * Member function definitions of Kalman template class.
 * See kalman.h for template class declaration.
 */

namespace observer {

template <typename T>
Kalman<T>::Kalman(T& model) : Observer<T>(model, state_t::Zero()) {
    reset();
}

template <typename T>
Kalman<T>::Kalman(T& model, const state_t& x0) : Observer<T>(model, x0) {
    reset();
}

template <typename T>
Kalman<T>::Kalman(T& model, const state_t& x0,
        const process_noise_covariance_t& Q,
        const measurement_noise_covariance_t& R,
        const error_covariance_t& P0) : Observer<T>(model, x0),
            m_P(P0), m_Q(Q), m_R(R) {
    m_K.setZero();
}

template <typename T>
void Kalman<T>::reset() {
    m_x.setZero();
    m_P.setIdentity();
    m_Q.setIdentity();
    m_R.setIdentity();
    m_K.setZero();
}

template <typename T>
void Kalman<T>::update_state(const input_t& u, const measurement_t& z) {
    time_update(u);
    measurement_update(z);
}

template <typename T>
void Kalman<T>::time_update() {
    time_update_state();
    time_update_error_covariance();
}

template <typename T>
void Kalman<T>::time_update(const process_noise_covariance_t& Q) {
    time_update_state();
    time_update_error_covariance(Q);
}

template <typename T>
void Kalman<T>::time_update(const input_t& u) {
    time_update_state(u);
    time_update_error_covariance();
}

template <typename T>
void Kalman<T>::time_update(const input_t& u, const process_noise_covariance_t& Q) {
    time_update_state(u);
    time_update_error_covariance(Q);
}

template <typename T>
void Kalman<T>::measurement_update(const measurement_t& z) {
    measurement_update_kalman_gain();
    measurement_update_state(z);
    measurement_update_error_covariance();
}

template <typename T>
void Kalman<T>::measurement_update(const measurement_t& z, const measurement_noise_covariance_t& R) {
    measurement_update_kalman_gain(R);
    measurement_update_state(z);
    measurement_update_error_covariance();
}

template <typename T>
void Kalman<T>::time_update_state() {
    m_x = m_model.normalize_state(m_model.Ad()*m_x);
}

template <typename T>
void Kalman<T>::time_update_state(const input_t& u) {
    m_x = m_model.normalize_state(m_model.Ad()*m_x + m_model.Bd()*u);
}

template <typename T>
void Kalman<T>::time_update_error_covariance() {
    m_P = m_model.Ad()*m_P*m_model.Ad().transpose() + m_Q;
}

template <typename T>
void Kalman<T>::time_update_error_covariance(const process_noise_covariance_t& Q) {
    m_P = m_model.Ad()*m_P*m_model.Ad().transpose() + Q;
}

template <typename T>
void Kalman<T>::measurement_update_kalman_gain() {
    // S = C*P*C' + R
    // K = P*C'*S^-1 - > K' = S^-1*C*P'
    Eigen::LDLT<measurement_noise_covariance_t> S_ldlt(
            m_model.Cd()*m_P*m_model.Cd().transpose() + m_R);
    m_K.noalias() = S_ldlt.solve(m_model.Cd()*m_P.transpose()).transpose();
}

template <typename T>
void Kalman<T>::measurement_update_kalman_gain(const measurement_noise_covariance_t& R) {
    // S = C*P*C' + R
    // K = P*C'*S^-1 - > K' = S^-1*C*P'
    Eigen::LDLT<measurement_noise_covariance_t> S_ldlt(
            m_model.Cd()*m_P*m_model.Cd().transpose() + R);
    m_K.noalias() = S_ldlt.solve(m_model.Cd()*m_P.transpose()).transpose();
}

template <typename T>
void Kalman<T>::measurement_update_state(const measurement_t& z) {
    m_x = m_model.normalize_state(m_x + m_K*(
                m_model.normalize_output(z - m_model.Cd()*m_x)));
}

template <typename T>
void Kalman<T>::measurement_update_error_covariance() {
    m_P = (error_covariance_t::Identity() - m_K*m_model.Cd())*m_P;
}

} // namespace observer
