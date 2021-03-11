/// \file fake_turtle.cpp
/// \brief contains a node called fake_turtle to simulate a differential drive robot
/// using the DiffDrive class
///
/// PARAMETERS:
///     left_wheel_joint : string used for publishing joint_state_message
///     right_wheel_joint : string used for publishing joint_state_message
///     wheelRad : the radius of the robot's wheels
///     wheelBase : the distance between the robot's wheels
/// PUBLISHES:
///     sensor_msgs/JointState on the joint state topic
/// SUBSCRIBES:
///     geometry_msgs/Twist on the cmd_vel topic
/// SERVICES:

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

#include <string>
#include <random>
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>

/****************************
* Declaring global variables
****************************/
static geometry_msgs::Twist twist_msg;
static const double PI = 3.14159265359;

/****************************
* Declare helper functions
****************************/
void twistCallback(const geometry_msgs::Twist msg);

/****************************
 * get_random() function
 * *************************/
std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

/****************************
* Main Function
****************************/
int main(int argc, char* argv[])
{
    using namespace rigid2d;

    /**********************
    * Initialize the node & node handle
    **********************/
    ros::init(argc, argv, "tube_world");
    ros::NodeHandle n;

    /**********************
    * Initialize local variables
    **********************/
    int frequency = 10;
    double wheelBase, wheelRad, slip_min, slip_max, twist_noise, tube_rad, max_range, tube_var;
    double slip_mean = (slip_min + slip_max) / 2;
    double slip_var = slip_max - slip_mean;
    double slip_noiseL, slip_noiseR;

    std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
    std::string world_frame_id, turtle_frame_id;

    sensor_msgs::JointState joint_msg;
    visualization_msgs::Marker marker1;
    visualization_msgs::Marker marker2;
    // visualization_msgs::MarkerArray markerArray;

    nav_msgs::Path path;

    wheelVel wheelVelocities;

    std::vector<double> tube1_loc;
    std::vector<double> tube2_loc;

    /**********************
    * Reads parameters from parameter server
    **********************/
    n.getParam("wheel_base", wheelBase);
    n.getParam("wheel_radius", wheelRad);
    n.getParam("odom_frame_id", odom_frame_id);
    n.getParam("body_frame_id", body_frame_id);
    n.getParam("left_wheel_joint", left_wheel_joint);
    n.getParam("right_wheel_joint", right_wheel_joint);
    n.getParam("slip_min", slip_min);
    n.getParam("slip_max", slip_max);
    n.getParam("twist_noise", twist_noise);
    n.getParam("tube1_location", tube1_loc);
    n.getParam("tube2_location", tube2_loc);
    n.getParam("tube_radius", tube_rad);
    n.getParam("max_range", max_range);
    n.getParam("world_frame_id", world_frame_id);
    n.getParam("turtle_frame_id", turtle_frame_id);
    n.getParam("tube_var", tube_var);

    /**********************
     * Initialize more local variables
     * *******************/
    std::normal_distribution<> g_vx(0, twist_noise);
    std::normal_distribution<> g_th(0, twist_noise);
    std::normal_distribution<> slip_noise(slip_mean, slip_var);
    std::normal_distribution<> tube_noise(0, tube_var);

    /**********************
    * Define publisher, subscriber, service and clients
    **********************/
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/joint_states", frequency);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/fake_sensor", 10, true);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/real_path", frequency);

    ros::Publisher test_marker_pub = n.advertise<visualization_msgs::Marker>("/fake_marker", 10, true);

    ros::Subscriber sub = n.subscribe("/cmd_vel", frequency, twistCallback);
    tf2_ros::TransformBroadcaster broadcaster;

    ros::Rate loop_rate(frequency);

    /**********************
    * Set initial parameters of the differential drive robot to 0
    * Set the initial position of the left and right wheel
    **********************/
    DiffDrive ninjaTurtle = DiffDrive(wheelBase, wheelRad, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_msg.name.push_back(left_wheel_joint);
    joint_msg.name.push_back(right_wheel_joint);

    joint_msg.position.push_back(0.0);
    joint_msg.position.push_back(0.0);

    pub.publish(joint_msg);

    // markerArray.markers.push_back(marker1);
    // markerArray.markers.push_back(marker2);

    ros::Time last_time = ros::Time::now();
    ros::Time current_time;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::spinOnce();

        /**********************
         * Publish cylindrical markers corresponding to locations of the tubes
         * *******************/
        
        visualization_msgs::MarkerArray markerArray;

        // marker1

        // locations of tubes should be relative to the location of the robot
        marker1.header.frame_id = world_frame_id;
        marker1.header.stamp = current_time;
        marker1.ns = "real";
        marker1.id = 0;
        marker1.type = visualization_msgs::Marker::CYLINDER;
        if (sqrt(pow(tube1_loc[0] - ninjaTurtle.getX(), 2) + pow(tube1_loc[1] - ninjaTurtle.getY(), 2)) > max_range)
        {
            marker1.action = visualization_msgs::Marker::DELETE;
        } else
        {
            marker1.action = visualization_msgs::Marker::ADD;
        }

        tf2::Quaternion marker_quat;
        marker_quat.setRPY(0, 0, 0);

        geometry_msgs::Quaternion markerQuat = tf2::toMsg(marker_quat);

        marker1.pose.position.x = -.5;
        marker1.pose.position.y = -.5;
        marker1.pose.position.z = 1.0;
        marker1.pose.orientation = markerQuat;
        marker1.scale.x = tube_rad;
        marker1.scale.y = tube_rad;
        marker1.scale.z = 0.2;
        marker1.color.a = 0.5;
        marker1.color.r = 255/255;
        marker1.color.g = 192/255;
        marker1.color.b = 203/255;

        marker1.frame_locked = true;

        markerArray.markers.push_back(marker1);

        test_marker_pub.publish(marker1);

        // marker2
        marker2.header.frame_id = "world";
        marker2.header.stamp = current_time;
        marker2.ns = "real";
        marker2.id = 1;
        marker2.type = visualization_msgs::Marker::CYLINDER;
        if (sqrt(pow(tube2_loc[0] - ninjaTurtle.getX(), 2) + pow(tube2_loc[1] - ninjaTurtle.getY(), 2)) > max_range)
        {
            marker2.action = visualization_msgs::Marker::DELETE;
        } else
        {
            marker2.action = visualization_msgs::Marker::ADD;
        }
        marker2.pose.position.x = 0.5;
        marker2.pose.position.y = 0.5;
        marker2.pose.position.z = 1.0;
        marker2.pose.orientation = markerQuat;
        marker2.scale.x = tube_rad;
        marker2.scale.y = tube_rad;
        marker2.scale.z = 0.2;
        marker2.color.a = 0.5;
        marker2.color.r = 255/255;
        marker2.color.g = 192/255;
        marker2.color.b = 203/255;

        marker2.frame_locked = true;

        markerArray.markers.push_back(marker2);

        marker_pub.publish(markerArray);

        /**********************
        * Create the desired twist based on the twist message
        **********************/
        Twist2D desiredTwist;
        desiredTwist.dth = twist_msg.angular.z;
        desiredTwist.dx = twist_msg.linear.x;
        desiredTwist.dy = twist_msg.linear.y;

        // Add Gaussian noise to the commanded twist
        desiredTwist.dth += g_th(get_random());
        desiredTwist.dx += g_vx(get_random());

        /**********************
        * Find the wheel velocities required to achieve that twist
        **********************/
        wheelVelocities = ninjaTurtle.convertTwist(desiredTwist);

        /**********************
        * Populate the sensor messages
        **********************/
        joint_msg.header.stamp = current_time;
        joint_msg.header.frame_id = odom_frame_id;

        joint_msg.position[0] += wheelVelocities.uL * (current_time - last_time).toSec();
        joint_msg.position[1] += wheelVelocities.uR * (current_time - last_time).toSec();

        // Add wheel slip noise
        slip_noiseL = slip_noise(get_random()) * wheelVelocities.uL;
        slip_noiseR = slip_noise(get_random()) * wheelVelocities.uR;

        // joint_msg.position[0] += slip_noiseL;
        // joint_msg.position[1] += slip_noiseR;

        /**********************
         * Update the configuration of the diff drive based on new wheel angles
         * *******************/
        ninjaTurtle(joint_msg.position[0], joint_msg.position[1]);
        
        pub.publish(joint_msg);

        /**********************
         * Publish a transform between the world and turtle frame
         * *******************/
        tf2::Quaternion odom_quater;
        odom_quater.setRPY(0, 0, ninjaTurtle.getTh());

        geometry_msgs::Quaternion odom_quat = tf2::toMsg(odom_quater);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = world_frame_id;
        odom_trans.child_frame_id = turtle_frame_id;

        odom_trans.transform.translation.x = ninjaTurtle.getX();
        odom_trans.transform.translation.y = ninjaTurtle.getY();
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        broadcaster.sendTransform(odom_trans);

        /**********************
         * Publish a message to show the actual robot trajectory
         * *******************/
        geometry_msgs::PoseStamped poseStamp;
        path.header.stamp = current_time;
        path.header.frame_id = world_frame_id;
        poseStamp.pose.position.x = ninjaTurtle.getX();
        poseStamp.pose.position.y = ninjaTurtle.getY();
        poseStamp.pose.orientation.z = ninjaTurtle.getTh();

        path.poses.push_back(poseStamp);
        path_pub.publish(path);

        /*********************
         * Update time and sleep
         * ******************/
        last_time = current_time;
        loop_rate.sleep();
    }
    return 0;
}

/// \brief twistCallback function
/// \param msg a geometry twist message
void twistCallback(const geometry_msgs::Twist msg)
{
    twist_msg = msg;
    return;
}

