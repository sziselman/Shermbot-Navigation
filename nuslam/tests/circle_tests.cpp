#include <catch_ros/catch.hpp>
#include <nuslam/circle_fit_library.hpp>
#include <armadillo>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <vector>

/// \brief testing circle fitting algorithm 
TEST_CASE("circle fit test 1", "[circle fit test 1]")
{
    using namespace arma;
    using namespace circle_fit;
    
    geometry_msgs::Point p1, p2, p3, p4, p5, p6;

    p1.x = 1;
    p1.y = 7;

    p2.x = 2;
    p2.y = 6;

    p3.x = 5;
    p3.y = 8;

    p4.x = 7;
    p4.y = 7;

    p5.x = 9;
    p5.y = 5;

    p6.x = 3;
    p6.y = 7;

    std::vector<geometry_msgs::Point> data{p1, p2, p3, p4, p5, p6};

    visualization_msgs::Marker marker = circleFit(data);

    REQUIRE(marker.pose.position.x == Approx(4.615482));
    REQUIRE(marker.pose.position.y == Approx(2.807354));
    REQUIRE(marker.scale.x == Approx(4.827575));
}

/// \brief testing circle fitting algorithm
TEST_CASE("circle fit test 2", "[circle fit test 2]")
{
    using namespace arma;
    using namespace circle_fit;

    geometry_msgs::Point p1, p2, p3, p4;

    p1.x = -1;
    p1.y = 0;

    p2.x = -0.3;
    p2.y = -0.06;

    p3.x = 0.3;
    p3.y = 0.1;

    p4.x = 1;
    p4.y = 0;

    std::vector<geometry_msgs::Point> data{p1, p2, p3, p4};

    visualization_msgs::Marker marker = circleFit(data);

    REQUIRE(marker.pose.position.x == Approx(0.4908357));
    REQUIRE(marker.pose.position.y == Approx(-22.15212));
    REQUIRE(marker.scale.x == Approx(22.17979));
}