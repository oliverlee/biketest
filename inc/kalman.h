#pragma once
#include <Eigen/Core>
#include "observer.h"

namespace observer {

/*
 * This template class implements a discrete-time Kalman Filter.
 */
template <typename Model>
class Kalman final : public Observer<Model> {
    public:
        /*
         * Although model_t, state_t, input_t, measurement_t are declared in the Observer<Model> class
         * and are inherited, the type aliases are dependent names as the base class (Observer)
         * is a template dependent on the template parameter of Kalman. In order to refer to
         * the type alias, we would need to specify 'typename Kalman::state_t', thus it is
         * easier to simply redeclare the type aliases.
         */
        using model_t = Model;
        using state_t = typename Model::state_t;
        using input_t = typename Model::input_t;
        using measurement_t = typename Model::output_t;
        using kalman_gain_t = typename Eigen::Matrix<real_t, Model::n, Model::l>;
        using error_covariance_t = typename Model::state_matrix_t;
        using process_noise_covariance_t = typename Model::state_matrix_t;
        using measurement_noise_covariance_t = typename Eigen::Matrix<real_t, Model::l, Model::l>;

        Kalman(Model& model);
        Kalman(Model& model, const state_t& x0);
        Kalman(Model& model, const state_t& x0,
                const process_noise_covariance_t& Q,
                const measurement_noise_covariance_t& R,
                const error_covariance_t& P0);

        virtual void reset() override;
        // simplified time and measurement update
        virtual void update_state(const input_t& u, const measurement_t& z) override;

        void time_update();
        void time_update(const process_noise_covariance_t& Q);
        void time_update(const input_t& u);
        void time_update(const input_t& u, const process_noise_covariance_t& Q);
        void measurement_update(const measurement_t& z);
        void measurement_update(const measurement_t& z, const measurement_noise_covariance_t& R);

        void set_x(const state_t& x);
        void set_P(const error_covariance_t& P);
        void set_Q(const process_noise_covariance_t& Q);
        void set_R(const measurement_noise_covariance_t& R);

        // accessors
        const state_t& x() const;
        const kalman_gain_t& K() const;
        const error_covariance_t& P() const;
        const process_noise_covariance_t& Q() const;
        const measurement_noise_covariance_t& R() const;

    private:
        using Observer<Model>::m_model;
        using Observer<Model>::m_x;
        kalman_gain_t m_K;
        error_covariance_t m_P;
        process_noise_covariance_t m_Q;
        measurement_noise_covariance_t m_R;

        void time_update_state();
        void time_update_state(const input_t& u);
        void time_update_error_covariance();
        void time_update_error_covariance(const process_noise_covariance_t& Q);
        void measurement_update_kalman_gain();
        void measurement_update_kalman_gain(const measurement_noise_covariance_t& R);
        void measurement_update_state(const measurement_t& z);
        void measurement_update_error_covariance();
}; // class Kalman

template <typename Model>
inline void Kalman<Model>::set_x(const state_t& x) {
    this->set_state(x);
}

template <typename Model>
inline void Kalman<Model>::set_P(const error_covariance_t& P) {
    m_P = P;
}

template <typename Model>
inline void Kalman<Model>::set_Q(const process_noise_covariance_t& Q) {
    m_Q = Q;
}

template <typename Model>
inline void Kalman<Model>::set_R(const measurement_noise_covariance_t& R) {
    m_R = R;
}

template <typename Model>
inline const typename Kalman<Model>::state_t& Kalman<Model>::x() const {
    return this->state();
}

template <typename Model>
inline const typename Kalman<Model>::kalman_gain_t& Kalman<Model>::K() const {
    return m_K;
}

template <typename Model>
inline const typename Kalman<Model>::error_covariance_t& Kalman<Model>::P() const {
    return m_P;
}

template <typename Model>
inline const typename Kalman<Model>::process_noise_covariance_t& Kalman<Model>::Q() const {
    return m_Q;
}

template <typename Model>
inline const typename Kalman<Model>::measurement_noise_covariance_t& Kalman<Model>::R() const {
    return m_R;
}

} // namespace observer

#include "kalman.hh"
