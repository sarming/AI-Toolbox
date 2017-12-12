#define BOOST_TEST_MODULE POMDP_PMDPGrid
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <vector>
#include <iostream>

#include <AIToolbox/POMDP/Algorithms/Witness.hpp>
#include <AIToolbox/POMDP/Algorithms/IncrementalPruning.hpp>
#include <AIToolbox/POMDP/Algorithms/PBVI.hpp>
#include <AIToolbox/POMDP/Types.hpp>
#include <AIToolbox/POMDP/Policies/Policy.hpp>
#include <AIToolbox/POMDP/IO.hpp>

#include "Utils/PMDP.hpp"
/* #include "../MDP/Utils/CornerProblem.hpp" */
#include "../MDP/Utils/GridWorld.hpp"
/* template <typename T> */
/* ostream& operator<<(ostream& output, std::vector<T> const& values) */
/* { */
/*         for (auto const& value : values) */
/*                 { */
/*                             output << value << std::endl; */
/*                                 } */
/*             return output; */
/* } */

auto makeCornerProblem(const GridWorld & grid, double stepUncertainty = 0.8) {
    using namespace AIToolbox::MDP;

    size_t S = grid.getSizeX() * grid.getSizeY(), A = 4;

    AIToolbox::Table3D transitions(boost::extents[S][A][S]);
    AIToolbox::Table3D rewards(boost::extents[S][A][S]);

    for ( size_t x = 0; x < grid.getSizeX(); ++x ) {
        for ( size_t y = 0; y < grid.getSizeY(); ++y ) {
            auto s = grid(x,y);
            if ( s == 0 || s == S-1 || s == S/2) {
                // Self absorbing states
                for ( size_t a = 0; a < A; ++a )
                    transitions[s][a][s] = 1.0;
            }
            else {
                for ( size_t a = 0; a < A; ++a ) {
                    auto st = grid(s), sl=st, sr=st;
                    st.setAdjacent((Direction)a);
                    sl.setLeft((Direction)a);
                    sr.setRight((Direction)a);
                    transitions[s][a][st] = stepUncertainty;
                    transitions[s][a][sl] += (1.0 - stepUncertainty)/2;
                    transitions[s][a][sr] += (1.0 - stepUncertainty)/2;

                    rewards[s][a][S-1]=1.0;
                }
            }
        }
    }

    Model model(S, A, transitions, rewards);

    return model;
}

auto makeGrid(int x, int y, std::vector<double> parameter_points) {
    using namespace AIToolbox;

    GridWorld grid(x, y);
    std::vector<MDP::Model> mdps;
    for(auto p: parameter_points){
        auto mdp = makeCornerProblem(grid,p);
        /* MDP::operator<<(std::cout, mdp); */
        mdps.push_back(mdp);
    }
    auto model = makePMDP(mdps);

    return model;
}

BOOST_AUTO_TEST_CASE( witness ) {
    using namespace AIToolbox;

    /* auto model = makeGrid(3,3,{0.8,0.9}); */
    auto model = makeGrid(3,3,{0.3,0.5,0.8});
    model.setDiscount(0.95);
    /* POMDP::operator<<(std::cout, model); */
    /* std::cout << model.getTransitionFunction() << std::endl; */
    /* for (auto const& value : model.getTransitionFunction()) */
    /* { */
    /*         std::cout << value << std::endl; */
    /* } */
    unsigned horizon = 20;
    /* POMDP::Witness solver(horizon, 0.1); */
    /* POMDP::IncrementalPruning solver(horizon, 0.5); */
    POMDP::PBVI solver(2000, horizon, 0.1);
    auto solution = solver(model);
    auto & vf = std::get<1>(solution);
    auto siz = vf[horizon].size();
    POMDP::Policy policy(model.getS(), model.getA(), model.getO(), vf);

    std::cout <<policy << std::endl;
    std::cout << siz;
}
