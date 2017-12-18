#define BOOST_TEST_MODULE POMDP_PMDPGrid
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <vector>
#include <iostream>

#include <AIToolbox/POMDP/Algorithms/Witness.hpp>
#include <AIToolbox/POMDP/Algorithms/IncrementalPruning.hpp>
#include <AIToolbox/POMDP/Algorithms/PERSEUS.hpp>
#include <AIToolbox/POMDP/Algorithms/PBVI.hpp>
#include <AIToolbox/POMDP/Algorithms/Utils/Pruner.hpp>
#include <AIToolbox/POMDP/Algorithms/Utils/WitnessLP.hpp>
#include <AIToolbox/POMDP/Types.hpp>
#include <AIToolbox/POMDP/Policies/Policy.hpp>
#include <AIToolbox/POMDP/IO.hpp>

#include "Utils/LearnerProblem.hpp"

std::ostream& operator<<(std::ostream &os, const AIToolbox::POMDP::VEntry & vv) {
    using namespace AIToolbox::POMDP;
    // Values
    os << std::get<VALUES>(vv).transpose() << " A=";
    // Action
    os << std::get<ACTION>(vv) << " ";
    // Obs
    for ( const auto & o : std::get<OBS>(vv) )
        os << o << ' ';
    return os;
}

std::ostream& operator<<(std::ostream &os, const AIToolbox::POMDP::VList & vl) {
    size_t i = 0;
    for (const auto & vv : vl )
        os << i++ << ": "<<vv << std::endl;
    return os;
}

BOOST_AUTO_TEST_CASE( PBVI ) {
    using namespace AIToolbox;

    /* auto model = makeLearnerProblem({0.8}); */
    /* auto model = makeLearnerProblem({0.2,0.5,0.8}); */
    auto model = makeLearnerProblem( linspace(0.0,1.0,5) );
    unsigned horizon = 200;
    /* POMDP::Witness solver(horizon, 0.1); */
    /* POMDP::IncrementalPruning solver(horizon, 0.5); */
    POMDP::PBVI solver(2000, horizon, 0.0);
    auto solution = solver(model);

    /* POMDP::PERSEUS solver(2000, horizon, 0.0); */
    /* model.setDiscount(0.9999999); */
    /* auto solution = solver(model, 0.0); */

    std::cout << "Solution found\n";

    auto & vf = std::get<1>(solution);
    auto & vl = vf.back();
    POMDP::Pruner<POMDP::WitnessLP>(model.getS())(&vl);
    POMDP::Policy policy(model.getS(), model.getA(), model.getO(), vf);

    auto belief = makeUniformBelief(0, model);

    std::cout <<vl << "\n";
    double value;
    auto i = POMDP::findBestAtBelief(belief, begin(vl), end(vl), &value);
    std::cout << i-begin(vl) << "=" <<value <<":" << *i << std::endl;
}

