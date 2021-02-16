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

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

/******************
* Declare global variables
******************/
static ros::Publisher wheelCom_pub;
static ros::Publisher jointState_pub;

static rigid2d::DiffDrive ninjaTurtle;

static geometry_msgs::Twist twist_msg;
static nuturtlebot::SensorData sensor_data;

/******************
* Helper Functions
******************/
void twistCallback(const geometry_msgs::Twist msg);
void sensorCallback(const nuturtlebot::SensorData data);

int main(int argc, char* argv[])
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
    double maxAngVel = 57;
    double wheelRad = n.getParam("wheel_radius", wheelRad);
    double wheelBase = n.getParam("wheel_base", wheelBase);
    std::string left_wheel_joint;
    n.getParam("left_wheel_joint", left_wheel_joint);
    std::string right_wheel_joint;
    n.getParam("right_wheel_joint", right_wheel_joint);
    std::string odom_frame_id;
    n.getParam("odom_frame_id", odom_frame_id);

    sensor_msgs::JointState joint_msg;

    /**********************
    * Define publisher, subscriber, services and clients
    **********************/
    wheelCom_pub = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", frequency);
    jointState_pub = n.advertise<sensor_msgs::JointState>("/joint_states", frequency);

    ros::Subscriber twist_sub = n.subscribe("/cmd_vel", frequency, twistCallback);
    ros::Subscriber sensor_sub = n.subscribe("/sensor_data", frequency, sensorCallback);

    ros::Rate loop_rate(frequency);

    /*********************
    * Set initial parameters of the diff-drive robot
    *********************/
    ninjaTurtle = DiffDrive(wheelBase, wheelRad, 0.0, 0.0, 0.0, 0.0, 0.0);


    joint_msg.name.push_back(left_wheel_joint);
    joint_msg.name.push_back(right_wheel_joint);

    joint_msg.position.push_back(0.0);
    joint_msg.position.push_back(0.0);

    joint_msg.velocity.push_back(0.0);
    joint_msg.velocity.push_back(0.0);
    
    jointState_pub.publish(joint_msg);

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();

        /********************
        * Get desird twist from twist message
        ********************/
        Twist2D desiredTwist;
        desiredTwist.dth = twist_msg.angular.z;
        desiredTwist.dx = twist_msg.angular.x;
        desiredTwist.dy = twist_msg.angular.y;

        /********************
        * Read the encoder data to update robot config based on current wheel angles
        ********************/
        double leftAngle = (2 * PI / 4096) * sensor_data.left_encoder;
        double rightAngle = (2 * PI / 4096) * sensor_data.right_encoder;

        ninjaTurtle(leftAngle, rightAngle);

        /********************
        * Get wheel velocities required to achieve desired twist
        ********************/
        wheelVel velocities = ninjaTurtle.convertTwist(desiredTwist);
        
        // Checks to make sure wheel velocities do not exceed maximum
        if (velocities.uL > maxAngVel)
        {
            velocities.uL = maxAngVel;
        } else if (velocities.uL < -maxAngVel)
        {
            velocities.uL = -maxAngVel;
        }

        if (velocities.uR > maxAngVel)
        {
            velocities.uL = maxAngVel;
        } else if (velocities.uR < -maxAngVel)
        {
            velocities.uR = -maxAngVel;
        }

        /********************
        * Update configuration of the diff drive robot
        ********************/
        double thLnew = ninjaTurtle.getThL() + velocities.uL;
        double thRnew = ninjaTurtle.getThR() + velocities.uR;

        // ninjaTurtle(thLnew, thRnew);

        /********************
        * publish wheel_cmd message
        ********************/
        nuturtlebot::WheelCommands wheelCom_msg;
        // converts the velocity to an integer value between -256 and 256 proportional to max rotational velocity
        int leftCommand = velocities.uL * (256/maxAngVel);
        int rightCommand = velocities.uR * (256/maxAngVel);

        wheelCom_msg.left_velocity = leftCommand;
        wheelCom_msg.right_velocity = rightCommand;
        wheelCom_pub.publish(wheelCom_msg);

        /******************
        * publish joint_states message
        ******************/
        joint_msg.header.stamp = current_time;
        joint_msg.header.frame_id = odom_frame_id;

        joint_msg.position[0] = ninjaTurtle.getThL();
        joint_msg.position[1] = ninjaTurtle.getThR();

        joint_msg.velocity[0] = velocities.uL;
        joint_msg.velocity[1] = velocities.uR;

        jointState_pub.publish(joint_msg);

        loop_rate.sleep();
    }
    return 0;
}

/// \brief twistCallback function
/// \param msg a geometry twist message
/// when cmd_vel message is received, it publishes wheel_cmd message
/// that makes the robot follow that twist
void twistCallback(const geometry_msgs::Twist msg)
{
    ROS_INFO("inside twist callback function");
    twist_msg = msg;
}

/// \brief sensorCallback function
/// \param msg
/// when sensor_data message is received, it publishes joint_states
/// message to provid the angle and velocity of the wheels, based on
/// encoder data
void sensorCallback(const nuturtlebot::SensorData data)
{
    ROS_INFO("inside sensor callback function");
    sensor_data = data;
}