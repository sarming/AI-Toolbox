#define BOOST_TEST_MODULE Experience
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <AIToolbox/Experience.hpp>

#include <array>
#include <algorithm>
#include <fstream>

BOOST_AUTO_TEST_CASE( construction ) {
    const int S = 5, A = 6;

    AIToolbox::Experience exp(S, A);

    BOOST_CHECK_EQUAL(exp.getS(), S);
    BOOST_CHECK_EQUAL(exp.getA(), A);

    BOOST_CHECK_EQUAL(exp.getVisit(0,0,0), 0);
    BOOST_CHECK_EQUAL(exp.getReward(0,0,0), 0.0);

    BOOST_CHECK_EQUAL(exp.getVisit(S-1,S-1,A-1), 0);
    BOOST_CHECK_EQUAL(exp.getReward(S-1,S-1,A-1), 0.0);
}

BOOST_AUTO_TEST_CASE( recording ) {
    const int S = 5, A = 6;

    AIToolbox::Experience exp(S, A);

    const int s = 3, s1 = 4, a = 5;
    const double rew = 7.4, negrew = -4.2, zerorew = 0.0;

    BOOST_CHECK_EQUAL(exp.getVisit(s,s1,a), 0);

    exp.record(s,s1,a,rew);

    BOOST_CHECK_EQUAL(exp.getVisit(s,s1,a), 1);
    BOOST_CHECK_EQUAL(exp.getReward(s,s1,a), rew);

    exp.reset();

    BOOST_CHECK_EQUAL(exp.getVisit(s,s1,a), 0);

    exp.record(s,s1,a,negrew);

    BOOST_CHECK_EQUAL(exp.getVisit(s,s1,a), 1);
    BOOST_CHECK_EQUAL(exp.getReward(s,s1,a), negrew);

    exp.record(s,s1,a,zerorew);

    BOOST_CHECK_EQUAL(exp.getVisit(s,s1,a), 2);
    BOOST_CHECK_EQUAL(exp.getReward(s,s1,a), negrew);
}

int generator() {
    static int counter = 0;
    return ++counter;
}

BOOST_AUTO_TEST_CASE( compatibility ) {
    const int S = 4, A = 3;
    AIToolbox::Experience exp(S,A);

    std::array<std::array<std::array<int, A>, S>, S> values;
    for ( size_t s = 0; s < S; s++ ) 
        for ( size_t s1 = 0; s1 < S; s1++ ) 
            std::generate(values[s][s1].begin(), values[s][s1].end(), generator);

    exp.setVisits(values);
    exp.setRewards(values);

    for ( size_t s = 0; s < S; s++ ) 
        for ( size_t s1 = 0; s1 < S; s1++ ) 
            for ( size_t a = 0; a < A; a++ ) {
                BOOST_CHECK_EQUAL( exp.getVisit(s,s1,a), values[s][s1][a] );
                BOOST_CHECK_EQUAL( exp.getReward(s,s1,a), values[s][s1][a] );
            }
}

BOOST_AUTO_TEST_CASE( files ) {
    const int S = 96, A = 2;
    AIToolbox::Experience exp(S,A);
    {
        std::ifstream savedExp("./data/experience.txt");

        if ( !savedExp ) BOOST_FAIL("Data to perform test could not be loaded.");
        BOOST_CHECK( savedExp >> exp );
    }
    {
        std::ofstream output("./loadedExperience.txt");
        if ( !output ) BOOST_FAIL("Could not open file for writing.");
        BOOST_CHECK( output << exp );
    }
    {
        std::ifstream testExp("./loadedExperience.txt");
        std::ifstream savedExp("./data/experience.txt");

        std::stringstream testBuffer, trueBuffer;

        testBuffer << testExp.rdbuf();
        trueBuffer << savedExp.rdbuf();

        BOOST_CHECK( testBuffer.str() == trueBuffer.str() );
    }
}