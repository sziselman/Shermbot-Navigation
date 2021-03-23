/// \file landmarks.cpp
/// \brief contains a node called landmarks to detect landmarks and publish their relative locations
/// 
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVICES:

#include <ros/ros.h>

#include <nuslam/circle_fit_library.hpp>

#include <armadillo>


/********
 * Main Function
 * *****/
int main(int argc, char* argv[])
{
    using namespace arma;
    using namespace circle_fit;

    /********
     * Initialize the node & node handle
     * *****/
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle n;

    /********
     * Declare local variables
     * *****/
    int frequency = 100;

    ros::Rate loop_rate(frequency);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}