/// \file follow_circle.cpp
/// \brief contains a node that publishes commands to have the robot drive in a circle
/// of a specified radius at a specified speed
///
/// PARAMETERS:
///         radius : the radius of the circle that the robot drives in
///         speed : the speed of the robot
///         wheelBase : the distance between the wheels of the robot
///         wheelRad : the radius of the robot's wheels
/// PUBLISHES:
///         geometry_msgs/Twist : contains the linear x, y and angular z velocities
/// SUBSCRIBES:
/// SERVICES:
///         /control : tells the robot to move in counter-clockwise, clockwise or stop

#include <ros/ros.h>
#include <nuturtle_robot/control.h>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <geometry_msgs/Twist.h>

#include <string>

/***************
 * Declare global variables
 * ************/
static double radius;
static double speed;
static double wheelBase;
static double wheelRad;

/***************
 * Initialize global publisher, subscriber, services and clients
 * ************/
static ros::ServiceServer control_service;
static ros::ServiceClient control_client;

static ros::Publisher twist_pub;

/***************
 * Declare states that the turtlebot can be in
 * ************/
enum State {Idle, ccw, cw, stop};

static State current;

/***************
 * Helper Functions
 * ************/
bool control(nuturtle_robot::control::Request &req, nuturtle_robot::control::Response &res);

int main(int argc, char* argv[])
{
    using namespace rigid2d;

    /****************
     * Initialize node & node handler
    ****************/
    ros::init(argc, argv, "follow_circle");
    ros::NodeHandle n("~");

    /****************
     * Define variables
    ****************/
    int frequency = 100;
    n.getParam("radius", radius);
    n.getParam("speed", speed);
    n.getParam("wheel_base", wheelBase);
    n.getParam("wheel_radius", wheelRad);

    /****************
     * Define publisher, subscriber, services and clients
     * *************/
    twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", frequency);

    control_service = n.advertiseService("/control", control);
    control_client = n.serviceClient<nuturtle_robot::control>("/control");

    geometry_msgs::Twist twist_msg;

    current = Idle;

    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();

        switch (current)
        {
            case Idle:
                twist_msg.linear.x = 0.0;
                twist_msg.linear.y = 0.0;
                twist_msg.angular.z = 0.0;
                twist_pub.publish(twist_msg);
                break;

            case cw:
                twist_msg.angular.z = -speed / radius;
                twist_msg.linear.x = -speed;
                twist_msg.linear.y = 0.0;
                twist_pub.publish(twist_msg);
                break;

            case ccw:
                twist_msg.angular.z = speed / radius;
                twist_msg.linear.x = speed;
                twist_msg.linear.y = 0.0;
                twist_pub.publish(twist_msg);
                break;
                
            case stop:
                twist_msg.linear.x = 0.0;
                twist_msg.linear.y = 0.0;
                twist_msg.angular.z = 0.0;
                twist_pub.publish(twist_msg);
                break;
        }

        loop_rate.sleep();
    }
    return 0;
}

/// \brief control function for /control service
/// The service causes the robot to move counter-clockwise, clockwise or stop
/// \param req : The service request
/// \param res : The service response
/// \return true
bool control(nuturtle_robot::control::Request &req, nuturtle_robot::control::Response &res)
{
    if (req.direction == "ccw")
    {
        current = ccw;
    } else if (req.direction == "cw")
    {
        current = cw;
    } else if (req.direction == "stop")
    {
        current = stop;
    }
}