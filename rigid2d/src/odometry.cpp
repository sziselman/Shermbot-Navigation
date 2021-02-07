/// \file odometry.cpp
/// \brief contains a node called odometry that will publish odometry messages in a standard ROS way
///
/// PARAMETERS:
///     wheel_base (double) : The distance between wheels
///     wheel_radius (double)   : The radius of both wheels
///     odom_frame_id   : The name of the odometry tf frame
///     body_frame_id   : The name of the body tf frame
///     left_wheel_joint    : The name of the left wheel joint
///     right_wheel_joint   : The name of the right wheel joint
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVICES:

#include <ros/ros.h>
#include <rigid2d/set_pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

/****************************
* Initializing global publisher, subscriber, services, clients, etc.
****************************/

static ros::Publisher odom_pub;
static ros::Subscriber sub;

/****************************
* Main Function
****************************/
int main(int argc, char* argv[])
{
    /****************************
    * Initialize the node & node handle
    ****************************/
    ros::init(argc, argv, "odometry");
    ros::nodeHandle n;

    /****************************
    * Define publisher, subscriber, services and clients
    ****************************/
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    sub = n.subscribe("/sensor_msgs")

    ros::Rate loop_rate(10);

    ROS_INFO("hi");

}