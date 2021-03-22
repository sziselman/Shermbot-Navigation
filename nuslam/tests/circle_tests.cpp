#include <catch_ros/catch.hpp>
#include <nuslam/circle_fit_library.hpp>
#include <armadillo>

/// \brief testing circle fitting algorithm 
TEST_CASE("circle fit test 1", "[circle fit test 1]")
{
    using namespace arma;
    using namespace circle_fit;

    mat dataPoints(6, 2);

    dataPoints(0, 0) = 1;
    dataPoints(0, 1) = 7;

    dataPoints(1, 0) = 2;
    dataPoints(1, 1) = 6;

    dataPoints(2, 0) = 5;
    dataPoints(2, 1) = 8;
    
    dataPoints(3, 0) = 7;
    dataPoints(3, 1) = 7;
    
    dataPoints(4, 0) = 9;
    dataPoints(4, 1) = 5;
    
    dataPoints(5, 0) = 3;
    dataPoints(5, 1) = 7;

    CircleFit circle = CircleFit(dataPoints);

    REQUIRE(circle.getX() == Approx(4.615482));
    REQUIRE(circle.getY() == Approx(2.807354));
    REQUIRE(circle.getR() == Approx(4.8275));
}

/// \brief testing circle fitting algorithm
TEST_CASE("circle fit test 2", "[circle fit test 2]")
{
    using namespace arma;
    using namespace circle_fit;

    mat dataPoints(4, 2);

    dataPoints(0, 0) = -1;
    dataPoints(0, 1) = 0;

    dataPoints(1, 0) = -0.3;
    dataPoints(1, 1) = -0.06;

    dataPoints(2, 0) = 0.3;
    dataPoints(2, 1) = 0.1;

    dataPoints(3, 0) = 1;
    dataPoints(3, 1) = 0;

    CircleFit circle = CircleFit(dataPoints);

    REQUIRE(circle.getX() == Approx(0.4908357));
    REQUIRE(circle.getY() == Approx(-22.15212));
    REQUIRE(circle.getR() == Approx(22.17979));
}