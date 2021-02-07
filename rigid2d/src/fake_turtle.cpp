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

static const double PI = 3.14159265359;

/****************************
* Initializing global publisher, subscriber, services, clients, etc.
****************************/

static ros::Publisher fake_turtle_pub;
static ros::Subscriber fake_turtle_sub;

/****************************
* Main Function
****************************/
int main(int argc, char* argv[])
{
    /**********************
    * Initialize the node & node handle
    **********************/
    ros::init(argc, argv, "fake_turtle");
    ros::NodeHandle n;

    /**********************
    * Initialize local variables
    **********************/
    int frequency = 100;
    double wheelBase, wheelRad
    double x_i = 0;
    double y_i = 0;
    double th_i = 0;
    double left_i = 0;
    double right_i = 0;

    geometry_msgs::Twist twist_msg;
    sensor_msgs::JointState left_wheel;
    sensor_msgs::JointState right_wheel;

    rigid2d::DiffDrive fakeTurtle;
    rigid2d::Twist2D desiredTwist;
    rigid2d::wheelVel wheelVelocities;

    /**********************
    * Reads parameters from parameter server
    **********************/
    n.getParam("wheel_base", wheelBase);
    n.getParam("wheel_radius", wheelRad);

    /**********************
    * Define publisher, subscriber, service and clients
    **********************/
    left_pub = n.advertise<sensor_msgs::JointState("joint_state", frequency);
    right_pub = n.advertise<sensor_msgs::JointState("joint_state", frequency);
    fake_turtle_sub = n.subscribe("/cmd_vel", frequency, twistCallback);

    ros::Rate loop_rate(frequency)

    ROS_INFO("this is fake turtle node");

    /**********************
    * Set initial parameters of the differential drive robot to 0
    * Set the initial position of the left and right wheel
    **********************/
    fakeTurtle = DiffDrive(wheelBase, wheelRad, x_i, y_i, th_i, left_i, right_i);

    left_wheel.position = fakeTurtle.thL;
    right_wheel.position = fakeTurtle.thR;

    left_pub.publish(left_wheel);
    right_pub.publish(right_wheel);

    while (ros::ok())
    {
        ros::spinOnce()
        desiredTwist.dth = twist_msg.angular.z;
        desiredTwist.dx = twist_msg.linear.x;
        desiredTwist.dy = twist_msg.linear.y;

        wheelVelocities = convertTwistB(desiredTwist);      // finds the wheel velocities required to achieve desired twist

        left_wheel->position += wheelVelocities.uL;          // did not multiply by time, assuming dt is one time unit
        right_wheel->position += wheelVelocities.uR;

        fakeTurtle(left_wheel->position, right_wheel->position);    // update the configuration of the diff drive based on new wheel angles
        
        left_pub.publish(left_wheel);
        right_pub.publish(right_wheel);

    }
    return 0;
}

void twistCallback(const geometry_msgs::Twist & msg)
{
    ROS_INFO("x linear velocity:" << msg.linear.x);
    ROS_INFO("y linear velocity:" << msg.linear.y);
    ROS_INFO("z angular velocity:" << msg.angular.z)
    twist_msg = msg;
}

