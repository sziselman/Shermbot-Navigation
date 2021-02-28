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
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <random>
#include <iostream>
#include <math.h>

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
    ros::init(argc, argv, "fake_turtle");
    ros::NodeHandle n;

    /**********************
    * Initialize local variables
    **********************/
    int frequency = 1;
    double wheelBase, wheelRad, slip_min, slip_max, twist_noise, tube_rad, max_range, tube_var;
    double slip_mean = (slip_min + slip_max) / 2;
    double slip_var = slip_max - slip_mean;
    double slip_noiseL, slip_noiseR;

    std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
    std::string world_frame_id, turtle_frame_id;

    sensor_msgs::JointState joint_msg;
    visualization_msgs::MarkerArray marker1;
    // visualization_msgs::MarkerArray marker2;

    wheelVel wheelVelocities;

    std::vector<double> tube1_loc;
    // std::vector<double> tube2_loc;

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
    // n.getParam("tube2_location", tube2_loc);
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
    std::normal_distribution<> left_noise(slip_mean, slip_var);
    std::normal_distribution<> right_noise(slip_mean slip_var);

    /**********************
    * Define publisher, subscriber, service and clients
    **********************/
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/joint_states", frequency, 10000);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/fake_sensor", frequency, true)
    ros::Subscriber sub = n.subscribe("/cmd_vel", frequency, twistCallback);

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

    ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::spinOnce();

        /**********************
         * Publish cylindrical markers corresponding to locations of the tubes
         * *******************/
        if (sqrt( (tube1_loc[0] - )))

        /**********************
        * Create the desired twist based on the twist message
        **********************/
        Twist2D desiredTwist;
        desiredTwist.dth = twist_msg.angular.z;
        desiredTwist.dx = twist_msg.linear.x;
        desiredTwist.dy = twist_msg.linear.y;

        // Add Gaussian noise to the commanded twist
        desiredTwist.dth += d_th(get_random());
        desiredTwist.dx += d_vx(get_random());

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
        slip_noiseL = left_noise * wheelVelocities.uL;
        slip_noiseR = right_noise * wheelVelocities.uR;

        joint_msg.position[0] += slip_noiseL;
        joint_msg.position[1] += slip_noiseR;

        /**********************
         * Update the configuration of the diff drive based on new wheel angles
         * *******************/
        ninjaTurtle(joint_msg.position[0], joint_msg.position[1]);
        
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

