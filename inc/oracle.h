#pragma once
#include "observer.h"

namespace observer {

/*
 * This template class implements an observer that simply stores and returns
 * the system model state.
 */
template <typename Model>
class Oracle final : public Observer<Model> {
    public:
        /*
         * Although model_t, state_t, input_t, measurement_t are declared in the Observer<Model> class
         * and are inherited, the type aliases are dependent names as the base class (Observer)
         * is a template dependent on the template parameter of Oracle. In order to refer to
         * the type alias, we would need to specify 'typename Oracle::state_t', thus it is
         * easier to simply redeclare the type aliases.
         */
        using model_t = Model;
        using state_t = typename Model::state_t;
        using input_t = typename Model::input_t;
        using measurement_t = typename Model::output_t;

        Oracle(Model& model);
        Oracle(Model& model, const state_t& x0);

        virtual void reset() override;
        virtual void update_state(const input_t& u, const measurement_t& z) override;

    private:
        using Observer<Model>::m_model;
        using Observer<Model>::m_x;
};

} // namespace observer

#include "oracle.hh"
