#ifndef AI_TOOLBOX_POMDP_LEARNER_PROBLEM
#define AI_TOOLBOX_POMDP_LEARNER_PROBLEM

#include "PMDP.hpp"

#include <AIToolbox/MDP/Model.hpp>
#include <AIToolbox/POMDP/Model.hpp>

enum {
    A_A = 0,
    A_B  = 1,
    A_C  = 2
};

enum {
    S_0 = 0,
    S_A = 1,
    S_B = 2,
    S_C = 3,
    S_G = 4,
    S_X = 5
};

inline auto makeLearnerMDP(double param = 0.8) {
    using namespace AIToolbox;

    size_t S = 6, A = 3;

    Table3D transitions(boost::extents[S][A][S]);
    Table3D rewards(boost::extents[S][A][S]);

    transitions[S_0][A_C][S_A] = param;
    transitions[S_0][A_C][S_B] = 1.0 - param;
    transitions[S_A][A_C][S_C] = 1.0;
    transitions[S_B][A_C][S_C] = 1.0;

    /* transitions[S_C][A_C][S_X] = 1.0; */
    transitions[S_C][A_C][S_0] = 1.0;

    for (size_t s = 0; s < 3; ++s ) {
        transitions[s][A_A][S_X] = 1.0;
        transitions[s][A_B][S_X] = 1.0;
    }

    for (size_t a = 0; a < A; ++a ) {
        transitions[S_G][a][S_G] = 1.0;
        transitions[S_X][a][S_X] = 1.0;

        rewards[S_C][a][S_G] = 1.0;
    }

    transitions[S_C][A_A][S_G] = param;
    transitions[S_C][A_A][S_X] = 1.0 - param;

    transitions[S_C][A_B][S_G] = 1.0 - param;
    transitions[S_C][A_B][S_X] = param;

    MDP::Model model(S, A, transitions, rewards);
    return model;
}


inline auto makeLearnerProblem(std::vector<double> params = {0.2, 0.5, 0.8}) {
    using namespace AIToolbox;

    std::vector<MDP::Model> mdps;
    for(auto p: params)
        mdps.push_back( makeLearnerMDP(p) );

    auto model = makePMDP(mdps);
    return model;
}
#endif
