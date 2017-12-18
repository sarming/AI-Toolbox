#ifndef AI_TOOLBOX_POMDP_PMDP
#define AI_TOOLBOX_POMDP_PMDP

#include <AIToolbox/MDP/Model.hpp>
#include <AIToolbox/POMDP/Model.hpp>
#include <vector>

AIToolbox::POMDP::Model<AIToolbox::MDP::Model> makePMDP(std::vector<AIToolbox::MDP::Model> mdps) {
    size_t P = size(mdps);
    auto mdp = mdps[0];
    size_t S = mdp.getS() * P, A = mdp.getA(), O = mdp.getS();

    AIToolbox::POMDP::Model<AIToolbox::MDP::Model> model(O, S, A, mdp.getDiscount());

    AIToolbox::Table3D transitions(boost::extents[S][A][S]);
    AIToolbox::Table3D rewards(boost::extents[S][A][S]);
    AIToolbox::Table3D observations(boost::extents[S][A][O]);

    for(size_t p = 0; p < P; ++p){
        for ( size_t s = 0; s < O; ++s ) {
            for ( size_t a = 0; a < A; ++a ) {
                for ( size_t s1 = 0; s1 < O; ++s1 ) {
                    transitions[p*O+s][a][p*O+s1] = mdps[p].getTransitionProbability(s,a,s1);
                    rewards[p*O+s][a][p*O+s1] = mdps[p].getExpectedReward(s,a,s1);
                    observations[p*O+s][a][s1] = 0.0;
                }
                observations[p*O+s][a][s] = 1.0;
            }
        }
    }

    model.setTransitionFunction(transitions);
    model.setRewardFunction(rewards);
    model.setObservationFunction(observations);

    return model;
}

auto makeUniformBelief(size_t s0, AIToolbox::POMDP::Model<AIToolbox::MDP::Model> pmdp){
    size_t S = pmdp.getS(), O = pmdp.getO();
    double p = (double)O/S; // probability of each parameter point

    AIToolbox::POMDP::Belief b = AIToolbox::POMDP::Belief::Zero(S);

    for (auto s = s0; s < S; s+=O )
        b[s]=p;

    return b;
}

auto linspace(double l, double r, size_t n) {
    if ( n==1 && l!=r) throw std::invalid_argument("l!=r");
    if ( l > r) throw std::invalid_argument("l>r");

    std::vector<double> v;
    if ( n==0 ) return v;

    auto step = (r-l)/(n-1);
    for (; l<r; l+=step)
        v.push_back(l);
    v.push_back(r);

    return v;
}

#endif
