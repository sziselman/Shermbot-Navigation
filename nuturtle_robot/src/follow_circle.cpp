/// \file follow_circle.cpp
/// \brief contains a node that publishes commands to have the robot drive in a circle
/// of a specified radius at a specified speed
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVICES:

#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "follow_circle");
    ros::NodeHandle n;

    int frequency = 100;

    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}