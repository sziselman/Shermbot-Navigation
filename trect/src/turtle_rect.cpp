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
/// Roughly followed tutorials provided by turtlesim page on ros wiki

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include "std_msgs/String.h"

const double PI = 3.14159265359;


/************************************************************************
* Helper Function Declaration
************************************************************************/


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

/// \brief callback function for turtlesim pose Subscriber
void poseCallback(const turtlesim::Pose::ConstPtr & pose_msg);


/************************************************************************
* Main Function
************************************************************************/


int main(int argc, char* argv[])
{
    double max_xdot, max_wdot;
    int frequency;
    double speed, distance;

    geometry_msgs::Twist msg;
    turtlesim::Pose turtlesim_pose;
    
    // initialize ROS node "turtle_rect"
    ros::init(argc, argv, "turtle_rect");
    // initialize node handle
    ros::NodeHandle n;

    n.getParam("max_xdot", max_xdot);
    n.getParam("max_wdot", max_wdot);
    n.getParam("frequency", frequency);

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", frequency);
    ros::Subscriber sub = n.subscribe("/turtle1/pose", 10, poseCallback);

    ros::Rate loop_rate(10);

    ROS_INFO("max_xdot: %f max_wdot: %f frequency: %d", max_xdot, max_wdot, frequency);
    while (ros::ok())
    {
        ROS_INFO("hi");
        // pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    
    return 0;
}

/************************************************************************
* Helper Function Declarations
************************************************************************/


void poseCallback(const turtlesim::Pose::ConstPtr & pose_msg)
{
    ROS_INFO("Position of turtle is x: %f y:%f th: %f linVel: %f angVel:%f", pose_msg->x, pose_msg->y, pose_msg->theta, pose_msg->linear_velocity, pose_msg->angular_velocity);
}