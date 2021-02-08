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
#include <string>
#include <iostream>

/****************************
* Initializing global publisher, subscriber, services, clients, etc.
****************************/

// static ros::Publisher odom_pub;
// static ros::Subscriber left_sub;
// static ros::Subscriber right_sub;

static sensor_msgs::JointState joint_msg;

/****************************
* Declare helper funcions
****************************/
void jointStateCallback(const sensor_msgs::JointState msg);

// static double wheelBase, wheelRad;
/****************************
* Main Function
****************************/
int main(int argc, char* argv[])
{
    using namespace rigid2d;

    /****************************
    * Initialize the node & node handle
    ****************************/
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;

    /****************************
    * Initialize local variables
    ****************************/
    nav_msgs::Odometry odom_msg;

    int frequency = 100;
    double x_i = 0;
    double y_i = 0;
    double th_i = 0;
    double left_i = 0;
    double right_i = 0;
    double wheelBase, wheelRad;
    
    std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;

    /****************************
    * Reading parameters from parameter server
    ****************************/
    n.getParam("wheel_base", wheelBase);
    n.getParam("wheel_radius", wheelRad);
    n.getParam("odom_frame_id", odom_frame_id);
    n.getParam("body_frame_id", body_frame_id);
    n.getParam("left_wheel_joint", left_wheel_joint);
    n.getParam("right_wheel_joint", right_wheel_joint);

    /****************************
    * Define publisher, subscriber, services and clients
    ****************************/
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", frequency);
    ros::Subscriber joint_sub = n.subscribe("/joint_states", frequency, jointStateCallback);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate loop_rate(frequency);

    /****************************
    * Set initial parameters of the differential drive robot to 0
    ****************************/
    DiffDrive odom_diffdrive = DiffDrive(wheelBase, wheelRad, x_i, y_i, th_i, left_i, right_i);

    ROS_INFO("Hello");

    while (ros::ok())
    {
        ros::spinOnce();

        ros::Time current_time = ros::Time::now();
        /****************************
        * Update the internal odometry state
        ****************************/
        ROS_INFO("yellow");
        // Twist2D twist_vel = odom_diffdrive.getTwist(joint_msg.position[0], joint_msg.position[1]);
        odom_diffdrive(joint_msg->position[0], joint_msg->position[1]);
        ROS_INFO("blue");
        /****************************
        * Create a quaternion created from yaw
        ****************************/
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_diffdrive.getTh());
        /****************************
        * Publish the transform over tf
        ****************************/
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = body_frame_id;

        odom_trans.transform.translation.x = odom_diffdrive.getX();
        odom_trans.transform.translation.y = odom_diffdrive.getY();
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        /****************************
        * Publish the odometry message over ROS
        ****************************/
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = odom_frame_id;

        odom_msg.pose.pose.position.x = odom_diffdrive.getX();
        odom_msg.pose.pose.position.y = odom_diffdrive.getY();
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        odom_msg.child_frame_id = body_frame_id;
        odom_msg.twist.twist.linear.x = joint_msg.velocity[0];
        odom_msg.twist.twist.linear.y = joint_msg.velocity[1];
        odom_msg.twist.twist.linear.z = joint_msg.velocity[2];

        odom_pub.publish(odom_msg);
        loop_rate.sleep();
    }
    return 0;
}

void jointStateCallback(const sensor_msgs::JointState msg)
{
    joint_msg = msg;
}