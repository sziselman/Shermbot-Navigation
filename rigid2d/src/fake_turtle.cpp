/// \file fake_turtle.cpp
/// \brief contains a node called fake_turtle to simulate a differential drive robot
/// using the DiffDrive class
///
/// PARAMETERS:
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

static const double PI = 3.14159265359;

/****************************
* Initializing global publisher, subscriber, services, clients, etc.
****************************/

// static ros::Publisher left_pub;
// static ros::Publisher right_pub;
// static ros::Subscriber fake_turtle_sub;
static geometry_msgs::Twist twist_msg;

/****************************
* Declare helper functions
****************************/
void twistCallback(const geometry_msgs::Twist & msg);

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
    int frequency = 100;
    double wheelBase, wheelRad;
    double x_i = 0;
    double y_i = 0;
    double th_i = 0;
    double left_i = 0;
    double right_i = 0;

    std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;

    sensor_msgs::JointState joint_msg;
    // geometry_msgs::Twist twist_msg;

    Twist2D desiredTwist;
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
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/joint_states", frequency);
    ros::Subscriber sub = n.subscribe("/cmd_vel", frequency, twistCallback);

    ros::Rate loop_rate(frequency);

    /**********************
    * Set initial parameters of the differential drive robot to 0
    * Set the initial position of the left and right wheel
    **********************/
    DiffDrive fakeTurtle = DiffDrive(wheelBase, wheelRad, x_i, y_i, th_i, left_i, right_i);

    joint_msg.name.push_back(left_wheel_joint);
    joint_msg.name.push_back(right_wheel_joint);

    joint_msg.position.push_back(0.0);
    joint_msg.position.push_back(0.0);

    pub.publish(joint_msg);

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::spinOnce();

        /**********************
        * Create the desired twist based on the twist message
        **********************/
        desiredTwist.dth = twist_msg.angular.z;
        desiredTwist.dx = twist_msg.linear.x;
        desiredTwist.dy = twist_msg.linear.y;

        /**********************
        * Find the wheel velocities required to achieve that twist
        **********************/
        wheelVelocities = fakeTurtle.convertTwistB(desiredTwist);

        /**********************
        * Populate the sensor messages
        **********************/
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.position[0] += wheelVelocities.uL;
        joint_msg.position[1] += wheelVelocities.uR;

        joint_msg.velocity[0] = desiredTwist.dth;
        joint_msg.velocity[1] = desiredTwist.dx;
        joint_msg.velocity[2] = desiredTwist.dy;
        
        fakeTurtle(joint_msg.position[0], joint_msg.position[1]);    // update the configuration of the diff drive based on new wheel angles
        
        pub.publish(joint_msg);
    }
    return 0;
}

void twistCallback(const geometry_msgs::Twist & msg)
{
    twist_msg = msg;
}

