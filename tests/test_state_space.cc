#include <Eigen/Dense>
#include "gtest/gtest.h"
#include "bicycle.h"
#include "parameters.h"
#include "test_utilities.h"
#include "test_state_space.h"

/*
 * State space matrices generated using dtk.bicycle and scipy.
 * See generate_state_space.py
 */


namespace {
    const double dt = 1.0/200;
    model::Bicycle::state_matrix_t A;
    model::Bicycle::input_matrix_t B(
        (model::Bicycle::input_matrix_t() <<
            0.                ,  0.                ,
            0.                ,  0.                ,
            0.                ,  0.                ,
            0.0159349789179135, -0.1240920254115741,
           -0.1240920254115741,  4.3238401808042282).finished()
        );
    model::Bicycle::state_matrix_t Ad;
    model::Bicycle::input_matrix_t Bd;

    const double vw = 4.29238253634111; // forward speed [m/s]
    const double vc = 6.02426201538837; // forward speed [m/s]

    // These matrices are (obviously) not correct and are used only to
    // determine if discrete state space matrices are correctly looked up.
    const model::Bicycle::state_matrix_t Ad_vw(
            2 * model::Bicycle::state_matrix_t::Identity()
            );
    const model::Bicycle::input_matrix_t Bd_vw(
            3 * model::Bicycle::input_matrix_t::Identity()
            );
    const model::Bicycle::state_matrix_t Ad_vc(
            4 * model::Bicycle::state_matrix_t::Identity()
            );
    const model::Bicycle::input_matrix_t Bd_vc(
            5 * model::Bicycle::input_matrix_t::Identity()
            );

    const model::Bicycle::state_space_map_t state_space_map {
        {model::Bicycle::make_state_space_map_key(vw, dt),
            model::Bicycle::state_space_map_value_t(Ad_vw, Bd_vw)},
        {model::Bicycle::make_state_space_map_key(vc, dt),
            model::Bicycle::state_space_map_value_t(Ad_vc, Bd_vc)},
    };
} // namespace

TEST_F(StateSpaceTest, ContinuousV1) {
    bicycle->set_v(1.0, 0);

    A <<
    0.0000000000000000, 0.0000000000000000, 0.9324083493089740, 0.0000000000000000, 0.0745926679447179,
    0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000, 0.0000000000000000,
    0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000,
    0.0000000000000000, 9.4865338000460664, -1.4625257433243051, -0.1055224498056882, -0.3305153989923120,
    0.0000000000000000, 11.7154748079957685, 28.9264833312917631, 3.6768052333214327, -3.0848655274330694;

    EXPECT_TRUE(bicycle->A().isApprox(A)) << output_matrices(bicycle->A(), A);
    EXPECT_TRUE(bicycle->B().isApprox(B)) << output_matrices(bicycle->B(), B);
}

TEST_F(StateSpaceTest, ContinuousV3) {
    bicycle->set_v(3.0, 0);

    A <<
    0.0000000000000000, 0.0000000000000000, 2.7972250479269221, 0.0000000000000000, 0.0745926679447179,
    0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000, 0.0000000000000000,
    0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000,
    0.0000000000000000, 9.4865338000460664, -8.5921076477970253, -0.3165673494170646, -0.9915461969769359,
    0.0000000000000000, 11.7154748079957685, 13.1527626512942426, 11.0304156999642977, -9.2545965822992091;

    EXPECT_TRUE(bicycle->A().isApprox(A)) << output_matrices(bicycle->A(), A);
    EXPECT_TRUE(bicycle->B().isApprox(B)) << output_matrices(bicycle->B(), B);
}

TEST_F(StateSpaceTest, ContinuousV5) {
    bicycle->set_v(5.0, 0);

    A <<
    0.0000000000000000e+00, 0.0000000000000000e+00, 4.6620417465448698e+00, 0.0000000000000000e+00, 7.4592667944717930e-02,
    0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00, 0.0000000000000000e+00,
    0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00,
    0.0000000000000000e+00, 9.4865338000460664e+00, -2.2851271456742467e+01, -5.2761224902844106e-01, -1.6525769949615603e+00,
    0.0000000000000000e+00, 1.1715474807995768e+01, -1.8394678708700734e+01, 1.8384026166607164e+01, -1.5424327637165348e+01;

    EXPECT_TRUE(bicycle->A().isApprox(A)) << output_matrices(bicycle->A(), A);
    EXPECT_TRUE(bicycle->B().isApprox(B)) << output_matrices(bicycle->B(), B);
}

TEST_F(StateSpaceTest, DiscreteV1) {
    bicycle->set_v(1.0, dt);

    Ad <<
    1.0000000000000000e+00, 1.1150047433809632e-05, 4.6894277236451910e-03, 3.4999489288757183e-06, 3.8174051320656106e-04,
    0.0000000000000000e+00, 1.0001184820643081e+00, -1.8478167519170524e-05, 4.9988533321204650e-03, -4.1402267568149167e-06,
    0.0000000000000000e+00, 1.4642849817488363e-04, 1.0003596378458959e+00, 4.5963276543359894e-05, 4.9622093457528911e-03,
    0.0000000000000000e+00, 4.7373286374364838e-02, -7.4307138855974368e-03, 9.9957576800707704e-01, -1.6579041282911602e-03,
    0.0000000000000000e+00, 5.8570670758658606e-02, 1.4347204345110903e-01, 1.8386655631933688e-02, 9.8503669772459101e-01;

    Bd <<
    -1.1742732635708518e-07, 4.0941186716096291e-06,
    2.0001145816138571e-07, -1.5807242572795022e-06,
    -1.5420741274461165e-06, 5.3764780115010109e-05,
    8.0170391584997460e-05, -6.3821951352698199e-04,
    -6.1503818438800187e-04, 2.1450096478647790e-02;

    EXPECT_TRUE(bicycle->Ad().isApprox(Ad)) << output_matrices(bicycle->Ad(), Ad);
    EXPECT_TRUE(bicycle->Bd().isApprox(Bd)) << output_matrices(bicycle->Bd(), Bd);
}

TEST_F(StateSpaceTest, DISABLED_DiscreteV3) {
    bicycle->set_v(3.0, dt);

    Ad <<
    1.0000000000000000e+00, 1.1150047433809632e-05, 4.6894277236451910e-03, 3.4999489288757183e-06, 3.8174051320656106e-04,
    0.0000000000000000e+00, 1.0001184820643081e+00, -1.8478167519170524e-05, 4.9988533321204650e-03, -4.1402267568149167e-06,
    0.0000000000000000e+00, 1.4642849817488363e-04, 1.0003596378458959e+00, 4.5963276543359894e-05, 4.9622093457528911e-03,
    0.0000000000000000e+00, 4.7373286374364838e-02, -7.4307138855974368e-03, 9.9957576800707704e-01, -1.6579041282911602e-03,
    0.0000000000000000e+00, 5.8570670758658606e-02, 1.4347204345110903e-01, 1.8386655631933688e-02, 9.8503669772459101e-01;

    Bd <<
    -1.1742732635708518e-07, 4.0941186716096291e-06,
    2.0001145816138571e-07, -1.5807242572795022e-06,
    -1.5420741274461165e-06, 5.3764780115010109e-05,
    8.0170391584997460e-05, -6.3821951352698199e-04,
    -6.1503818438800187e-04, 2.1450096478647790e-02;

    EXPECT_TRUE(bicycle->Ad().isApprox(Ad)) << output_matrices(bicycle->Ad(), Ad);
    EXPECT_TRUE(bicycle->Bd().isApprox(Bd)) << output_matrices(bicycle->Bd(), Bd);
}

TEST_F(StateSpaceTest, DiscreteV5) {
    bicycle->set_v(5.0, dt);

    Ad <<
    1.0000000000000000e+00, 1.2049991484992133e-05, 2.3291048326765866e-02, 1.8462645918076634e-05, 4.1567060022420490e-04,
    0.0000000000000000e+00, 1.0001180700462440e+00, -2.8474586368268200e-04, 4.9929766799901984e-03, -2.0583494132583432e-05,
    0.0000000000000000e+00, 1.4630038234223096e-04, 9.9976730145466564e-01, 2.2402776466154750e-04, 4.8110697443882310e-03,
    0.0000000000000000e+00, 4.7124896630597990e-02, -1.1371723873036946e-01, 9.9710530689603383e-01, -8.2185377039953947e-03,
    0.0000000000000000e+00, 5.8489213351501479e-02, -9.3617401457300686e-02, 8.8474932659789590e-02, 9.2518956230185589e-01;

    Bd <<
    -1.2411629143016838e-07, 4.3377179681611336e-06,
    2.0326445533610386e-07, -1.6981861891088091e-06,
    -1.5058897428593093e-06, 5.2632958211780891e-05,
    8.2117225610236940e-05, -7.0858832804455312e-04,
    -5.9344551127057076e-04, 2.0774496614372074e-02;

    EXPECT_TRUE(bicycle->Ad().isApprox(Ad)) << output_matrices(bicycle->Ad(), Ad);
    EXPECT_TRUE(bicycle->Bd().isApprox(Bd)) << output_matrices(bicycle->Bd(), Bd);
}

TEST(StateSpace, LookupFound) {
    model::Bicycle bicycle0(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            vw, dt, &state_space_map);
    model::Bicycle bicycle1(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            vw, dt);

    EXPECT_FALSE(bicycle0.Ad().isApprox(bicycle1.Ad())) << output_matrices(bicycle0.Ad(), bicycle1.Ad());
    EXPECT_FALSE(bicycle0.Bd().isApprox(bicycle1.Bd())) << output_matrices(bicycle0.Bd(), bicycle1.Bd());
    EXPECT_TRUE(bicycle0.Ad().isApprox(Ad_vw)) << output_matrices(bicycle0.Ad(), Ad_vw);
    EXPECT_TRUE(bicycle0.Bd().isApprox(Bd_vw)) << output_matrices(bicycle0.Bd(), Bd_vw);

    bicycle0.set_v(vc, dt);
    bicycle1.set_v(vc, dt);

    EXPECT_FALSE(bicycle0.Ad().isApprox(bicycle1.Ad())) << output_matrices(bicycle0.Ad(), bicycle1.Ad());
    EXPECT_FALSE(bicycle0.Bd().isApprox(bicycle1.Bd())) << output_matrices(bicycle0.Bd(), bicycle1.Bd());
    EXPECT_TRUE(bicycle0.Ad().isApprox(Ad_vc)) << output_matrices(bicycle0.Ad(), Ad_vc);
    EXPECT_TRUE(bicycle0.Bd().isApprox(Bd_vc)) << output_matrices(bicycle0.Bd(), Bd_vc);
}

TEST(StateSpace, DISABLED_LookupNotFound) {
    double v = 1.0;
    model::Bicycle bicycle0(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            v, dt, &state_space_map);
    model::Bicycle bicycle1(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            v, dt);

    EXPECT_TRUE(bicycle0.Ad().isApprox(bicycle1.Ad())) << output_matrices(bicycle0.Ad(), bicycle1.Ad());
    EXPECT_TRUE(bicycle0.Bd().isApprox(bicycle1.Bd())) << output_matrices(bicycle0.Bd(), bicycle1.Bd());

    v = 5.0;
    bicycle0.set_v(v, dt);
    bicycle1.set_v(v, dt);

    EXPECT_TRUE(bicycle0.Ad().isApprox(bicycle1.Ad())) << output_matrices(bicycle0.Ad(), bicycle1.Ad());
    //TODO: This next statement fails
    EXPECT_TRUE(bicycle0.Bd().isApprox(bicycle1.Bd())) << output_matrices(bicycle0.Bd(), bicycle1.Bd());
}
