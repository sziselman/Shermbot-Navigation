/// \file odometer.cpp
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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

#include <string>
#include <iostream>

/****************************
* Declare global variables
****************************/
static sensor_msgs::JointState joint_msg;
static std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
static ros::Publisher odom_pub;
static ros::ServiceServer setPose_service;
static ros::ServiceClient setPose_client;

// static tf::TransformBroadcaster odom_broadcaster;

static double wheelBase, wheelRad;
static int frequency = 100;

static rigid2d::DiffDrive odom_diffdrive;

/****************************
* Declare helper funcions
****************************/
void jointStateCallback(const sensor_msgs::JointState msg);
bool setPose(rigid2d::set_pose::Request & req, rigid2d::set_pose::Response &res);

/****************************
* Main Function
****************************/
int main(int argc, char* argv[])
{
    using namespace rigid2d;

    /****************************
    * Initialize the node & node handle
    ****************************/
    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;

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
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", frequency);
    setPose_service = n.advertiseService("set_pose", setPose);
    setPose_client = n.serviceClient<rigid2d::set_pose>("set_pose");

    ros::Subscriber joint_sub = n.subscribe("/joint_states", frequency, jointStateCallback);
    ros::Rate loop_rate(frequency);

    /****************************
    * Set initial parameters of the differential drive robot to 0
    ****************************/
    odom_diffdrive = DiffDrive(wheelBase, wheelRad, 0.0, 0.0, 0.0, 0.0, 0.0);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}

/// \brief callback function for subscriber to joint state message
/// Sends an odometry message and broadcasts a tf transform to update the configuration of the robot
///
/// \param msg : the joint state message
void jointStateCallback(const sensor_msgs::JointState msg)
{
    using namespace rigid2d;

    static tf2_ros::TransformBroadcaster odom_broadcaster;

    ros::Time current_time = ros::Time::now();

    /***********************
    * Get the velocities from new wheel angles and update configuration
    ***********************/
    Twist2D twist_vel = odom_diffdrive.getTwist(msg.position[0], msg.position[1]);

    odom_diffdrive(msg.position[0], msg.position[1]);

    /***********************
    * Create a quaternion from yaw
    ***********************/
    // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_diffdrive.getTh());
    tf2::Quaternion odom_quater;
    odom_quater.setRPY(0, 0, odom_diffdrive.getTh());
    // odom_quater.setRPY(0, 0, 1);

    geometry_msgs::Quaternion odom_quat = tf2::toMsg(odom_quater);

    /***********************
    * Publish the transform over tf
    ***********************/
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = body_frame_id;

    odom_trans.transform.translation.x = odom_diffdrive.getX();
    odom_trans.transform.translation.y = odom_diffdrive.getY();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    /***********************
    * Publish the odometry message over ROS
    ***********************/
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.pose.pose.position.x = odom_diffdrive.getX();
    odom_msg.pose.pose.position.y = odom_diffdrive.getY();
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.child_frame_id = body_frame_id;
    odom_msg.twist.twist.linear.x = twist_vel.dx;
    odom_msg.twist.twist.linear.y = twist_vel.dy;
    odom_msg.twist.twist.linear.z = twist_vel.dth;

    odom_pub.publish(odom_msg);

    return ;
}

/// \brief setPose function for set_pose service
/// The service request provides the configuration of the robot
/// The location of the odometry is reset so the robot is at the requested configuration
/// \param req : The service request
/// \param res : The service reponse
/// \return true
bool setPose(rigid2d::set_pose::Request &req, rigid2d::set_pose::Response &res)
{
    using namespace rigid2d;

    /****************************
    * Configuration of the robot
    ****************************/
    double xNew = req.x;
    double yNew = req.y;
    double thNew = req.th;

    /****************************
    * Location of odometry reset so robot is at requested location
    * Replaces odom_diffdrive with a new configuration
    ****************************/
    odom_diffdrive = DiffDrive(wheelBase, wheelRad, xNew, yNew, thNew, 0.0, 0.0);

    return true;
}