/// \file turtle_rect.cpp 
/// \brief contains a node called turtle_rect
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     topic_name (topic_type): description of topic
/// SUBSCRIBES:
///     topic_name (topic_type): description of the topic
/// SERVICES:
///     service_name (service_type): description of the service
/// Followed tutorials provided by turtlesim page on ros wiki

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include "geometry_msgs/Twist.h"

/// \brief moves the turtle straight
/// \param speed - the speed that the turtle moves
/// \param distance - the distance that the turtle moves
/// \param isForward - true if turtle is moving forward, false if turtle is moving backward
void move(double speed, double distance, bool isForward);

/// \brief rotates the turtle
/// \param angular_speed - the speed at which the turtle rotates
/// \param angle - the angle that the turtle rotates
/// \param clockwise - true if turtle is rotating clockwise, false if turtle is rotating counter-clockwise
void rotate(double angular_speed, double angle, bool clockwise);

/// \brief converts an angle in degrees to radians
/// \param degrees - the angle in degrees
/// \return the angle in radians
double deg2rad(double degrees);


int main(int argc, char* argv[])
{
    double max_xdot, max_wdot, frequency;
    double speed, distance;

    // initialize ROS node "turtle_rect"
    ros::init(argc, argv, "turtle_rect");
    // initialize node handle
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", frequency);
    
    // if parameters exist, get them and log them as ROS_INFO messages
    if (ros::param::get("/max_xdot", max_xdot))
    {
        ros::param::get("/max_xdot", max_xdot);
        ROS_INFO("The maximum translational velocity of the robot is %f\n", max_xdot);
    }
    if (ros::param::get("/max_wdot", max_wdot))
    {
        ros::param::get("/max_wdot", max_wdot);
        ROS_INFO("The maximum rotational velocity of the robot is %f\n", max_wdot);
    }
    if (ros::param::get("/frequency", frequency))
    {
        ros::param::get("/frequency", frequency);
        ros::Rate rate(frequency);
        ROS_INFO("The frequency of the control loop is %f\n", frequency);
    }

    while(ros::ok()) 
    {
        // initialize the message
        geometry_msgs::Twist msg;
        // publish the message
        pub.publish(msg);

        // delays until next message
        rate.sleep();

    }
}

void move(double speed, double distance, bool isForward)
{
    using namespace std;
    
    geometry_msgs::Twist vel_msg;
}