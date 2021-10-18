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
#include <sensor_msgs/JointState.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <string>
#include <iostream>

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
* Main Function
****************************/
int main(int argc, char* argv[])
{
    using namespace rigid2d;

    /**********************
    * Initialize the node & node handle
    **********************/
    ros::init(argc, argv, "fake_turtle");
    ros::NodeHandle n;

    /**********************
    * Initialize local variables
    **********************/
    int frequency = 1;
    double wheelBase, wheelRad;

    std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;

    sensor_msgs::JointState joint_msg;

    wheelVel wheelVelocities;

    /**********************
    * Reads parameters from parameter server
    **********************/
    n.getParam("wheel_base", wheelBase);
    n.getParam("wheel_radius", wheelRad);
    n.getParam("odom_frame_id", odom_frame_id);
    n.getParam("body_frame_id", body_frame_id);
    n.getParam("left_wheel_joint", left_wheel_joint);
    n.getParam("right_wheel_joint", right_wheel_joint);

    /**********************
    * Define publisher, subscriber, service and clients
    **********************/
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/joint_states", frequency, 10000);
    ros::Subscriber sub = n.subscribe("/cmd_vel", frequency, twistCallback);

    ros::Rate loop_rate(frequency);

    /**********************
    * Set initial parameters of the differential drive robot to 0
    * Set the initial position of the left and right wheel
    **********************/
    DiffDrive fakeTurtle = DiffDrive(wheelBase, wheelRad, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_msg.name.push_back(left_wheel_joint);
    joint_msg.name.push_back(right_wheel_joint);

    joint_msg.position.push_back(0.0);
    joint_msg.position.push_back(0.0);

    pub.publish(joint_msg);

    ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::spinOnce();

        /**********************
        * Create the desired twist based on the twist message
        **********************/
        Twist2D desiredTwist;
        desiredTwist.dth = twist_msg.angular.z;
        desiredTwist.dx = twist_msg.linear.x;
        desiredTwist.dy = twist_msg.linear.y;

        /**********************
        * Find the wheel velocities required to achieve that twist
        **********************/
        wheelVelocities = fakeTurtle.convertTwist(desiredTwist);

        /**********************
        * Populate the sensor messages
        **********************/
        joint_msg.header.stamp = current_time;
        joint_msg.header.frame_id = odom_frame_id;

        joint_msg.position[0] += wheelVelocities.uL * (current_time - last_time).toSec();
        joint_msg.position[1] += wheelVelocities.uR * (current_time - last_time).toSec();
        
        fakeTurtle(joint_msg.position[0], joint_msg.position[1]);    // update the configuration of the diff drive based on new wheel angles
        
        pub.publish(joint_msg);
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

