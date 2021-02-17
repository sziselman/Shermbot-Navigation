/// \file follow_circle.cpp
/// \brief contains a node that publishes commands to have the robot drive in a circle
/// of a specified radius at a specified speed
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVICES:

#include <ros/ros.h>
#include <nuturtle_robot/control.h>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

#include <string>

/***************
 * Declare global variables
 * ************/
static double radius;
static double speed;
static int ccw = true;
static ros::ServiceServer control_service;
static ros::ServiceClient control_client;

static ros::Publisher wheelCommand_pub;

int main(int argc, char* argv[])
{
    /****************
     * Initialize node & node handler
    ****************/
    ros::init(argc, argv, "follow_circle");
    ros::NodeHandle n;

    /****************
     * Define variables
    ****************/
    int frequency = 100;
    n.getParam("radius", radius);
    n.getParam("speed", speed);

    /****************
     * Define publisher, subscriber, services and clients
     * *************/
    wheelCommand_pub = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", frequency);
    control_service = n.advertiseService("/control", control);
    control_client = n.serviceClient<nuturtle_robot::control>("/control");
    
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
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
        ccw = true;
    } else if (req.direction == 'cw')
    {
        ccw = false;
    } else if (req.direction == 'stop')
    {
        // stop the robot!!!!!
    }
}