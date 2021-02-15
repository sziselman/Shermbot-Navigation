/// \file turtle_interface.cpp
/// \brief contains a node called turtle_interface that will implement low-level control
/// and sensor routines. Reads in a twist and converts it to a specific wheel command that
/// makes the turtle move. Also, reads raw encoder data and outputs it as joint angles and
/// velocities.
///
/// PARAMETERS:
/// PUBLISHES:  wheel_cmd (nuturtlebot/WheelCommands)
///             joint_states (sensor_msgs/JointState)
/// SUBSCRIBES: cmd_vel (geometry_msgs/Twist)
///             sensor_data (nuturtlebot/SensorData)
/// SERVICES:

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.cpp>

/******************
* Declare global variables
******************/
ros::Publisher wheelCom_pub;
ros::Publisher jointState_pub;

rigid2d::DiffDrive ninjaTurtle;

/******************
* Helper Functions
******************/
void twistCallback(const geometry_msgs::Twist msg);
void sensorCallback(const nuturtlebot::SensorData data);

int main(argc, char* argv[])
{
    using namespace rigid2d;

    /**********************
    * Initialize the node & node handle
    **********************/
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle n;

    /**********************
    * Define local variables
    **********************/
    int frequency = 100;
    double wheelRad = n.getParam("wheel_radius", wheelRad);
    double wheelBase = n.getParam("wheel_base", wheelBase);

    /**********************
    * Define publisher, subscriber, services and clients
    **********************/
    wheelCom_pub = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", frequency);
    jointState_pub = n.advertise<sensor_msgs::JointState>("/joint_states", frequency);/SensorData.h>

    ros::Subscriber twist_sub = n.subscribe("/cmd_vel", frequency, twistCallback);
    ros::Subscriber sensor_sub = n.subscribe("/sensor_data", frequency, sensorCallback);

    ros::Rate loop_rate(frequency);

    /*********************
    * Set initial parameters of the diff-drive robot
    *********************/
    ninjaTurtle = DiffDrive(wheelBase, wheelRad, 0.0, 0.0, 0.0, 0.0, 0.0);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

    while
}

/// \brief twistCallback function
/// \param msg a geometry twist message
/// when cmd_vel message is received, it publishes wheel_cmd message
/// that makes the robot follow that twist
void twistCallback(const geometry_msgs::Twist msg)
{
    using namespace rigid2d;

    ROS_INFO("inside twist callback function");

    /********************
    * Convert twist to wheel velocities
    ********************/
    Twist2D desiredTwist;
    desiredTwist.dth = msg.angular.z;
    desiredTwist.dx = msg.linear.x;
    desiredTwist.dy = msg.linear.y
    
    wheelVel velocities = ninjaTurtle.convertTwist(desiredTwist);

    // Checks to make sure wheel velocities do not exceed maximum
    if (wheelVel.uL > 0.22)
    {
        wheelVel.uL = 0.22;
    } else if (wheelVel.uL < -0.22)
    {
        wheelVel.uL = -0.22;
    }

    if (wheelVel.uR > 0.22)
    {
        wheelVel.uL = 0.22;
    } else if (wheelVel.uR < -0.22)
    {
        wheelVel.uR = -0.22;
    }

    /********************
    * Update configuration of the diff drive robot
    ********************/
    double thLnew = ninjaTurtle.thL += uL;
    double thRnew = ninjaTurtle.thR += uR;

    ninjaTurtle(thLnew, thRnew);

    /********************
    * publish wheel_cmd message
    ********************/
    nuturtlebot::wheelCommands wheelCom_msg;
    // converts the velocity to an integer value between -256 and 256 proportional to max rotational velocity
    int leftCommand = wheelVel.uL * (0.22/256);
    int rightCommand = wheelVel.uR * (0.22/256);

    wheelCom_msg.left_velocity = leftCommand;
    wheelCom_msg.right_velocity = rightCommand;
    wheelCom_pub.publish(wheelCom_msg);
}

/// \brief sensorCallback function
/// \param msg
/// when sensor_data message is received, it publishes joint_states
/// message to provid the angle and velocity of the wheels, based on
/// encoder data
void sensorCallback(const nuturtlebot::SensorData data)
{
    ROS_INFO("inside sensor callback function");

    /******************
    * publish joint_states message
    ******************/
    sensor_msgs::JointState joint_msg;
    jointState_pub.publish(joint_msg);
}