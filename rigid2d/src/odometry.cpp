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
static ros::Subscriber joint_sub;
static double wheelBase, wheelRad;
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
    * Initialize local variables
    ****************************/
    sensor_msgs::JointState left_msg;
    sensor_msgs::JointState right_msg;
    nav_msgs::Odometry odom_msg;

    int frequency = 100;
    double x_i = 0;
    double y_i = 0;
    double th_i = 0;
    double left_i = 0;
    double right_i = 0;

    rigid2d::DiffDrive odom_diffdrive;

    /****************************
    * Reading parameters from parameter server
    ****************************/
    n.getParam("wheel_base", wheelBase);
    n.getParam("wheel_radius", wheelRad);

    /****************************
    * Define publisher, subscriber, services and clients
    ****************************/
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", frequency);
    tf::TransformBroadcaster odom_broadcaster;
    left_sub = n.subscribe("joint_states", jointStateCallback);
    right_sub = n.subscribe("joint_states", jointStateCallback);

    ros::Rate loop_rate(frequency);

    /****************************
    * Set initial parameters of the differential drive robot to 0
    ****************************/
    odom_diffdrive = DiffDrive(wheelBase, wheelRad, x_i, y_i, th_i, left_i, right_i);

    ROS_INFO("Hello");

    while (ros::ok())
    {
        ros::spinOnce();

    }
}

void jointStateCallback(const sensor_msgs::JointState msg)
{
    ROS_INFO("IN JOINT STATE CALLBACK");
}