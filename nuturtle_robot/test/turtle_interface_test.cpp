/// \brief turtle_interface_test.cpp 
/// test file for turtle_interface node

#include "catch_ros/catch.hpp"
#include "ros/ros.h"
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <rigid2d/rigid2d.hpp>

// #include <nuturtle_robot/turtle_interface.h>

static int called1 = false;
static int called2 = false;
static int called3 = false;

void callback1(const nuturtlebot::WheelCommands command)
{
    if (command.left_velocity != 0 || command.right_velocity != 0 || called1)
    {
        called1 = true;
        REQUIRE(command.left_velocity == 129);
        REQUIRE(command.right_velocity == 129);
    } else
    {
        REQUIRE(command.left_velocity == 0);
        REQUIRE(command.right_velocity == 0);
    }
}

void callback2(const nuturtlebot::WheelCommands command)
{
    if (command.left_velocity != 0.0 || command.right_velocity != 0.0 || called2)
    {
        called2 = true;
        REQUIRE(command.left_velocity == -103);
        REQUIRE(command.right_velocity == 103);
    } else 
    {
        REQUIRE(command.left_velocity == 0);
        REQUIRE(command.right_velocity == 0);
    }
}

void callback3(const sensor_msgs::JointState msg)
{
    if (msg.position[0] != 0.0 || msg.position[1] != 0.0 || called3)
    {
        called3 = true;
        REQUIRE(msg.position[0] == Approx(3.067961576));
        REQUIRE(msg.position[1] == Approx(1.533980788));
    } else
    {
        REQUIRE(msg.position[0] == 0.0);
        REQUIRE(msg.position[1] == 0.0);
    }
}

TEST_CASE("pure translation", "[pure translation]")
{
    // this test case publishes a cmd_vel message and
    // subscribes to the turtle_interface node to check 
    // proper wheel commands

    ros::NodeHandle n1;
    int frequency = 100;

    const auto pub1 = n1.advertise<geometry_msgs::Twist>("/cmd_vel", frequency, true);

    const auto sub1 = n1.subscribe("/wheel_cmd", frequency, callback1);

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0.1;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    pub1.publish(twist_msg);

    ros::Rate loop_rate(frequency);
    for (int i = 0; ros::ok() && i != 200; ++i)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spinOnce();
}

TEST_CASE("pure rotation", "[pure rotation]")
{
    // this test case publishes a cmd_vel message and
    // subscribes to the turtle_interface node to check
    // proper wheel commands

    ros::NodeHandle n2;
    int frequency = 100;

    const auto pub2 = n2.advertise<geometry_msgs::Twist>("/cmd_vel", frequency, true);

    const auto sub2 = n2.subscribe("/wheel_cmd", frequency, callback2);

    geometry_msgs::Twist twist_msg; 
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 1;

    pub2.publish(twist_msg);

    ros::Rate loop_rate(frequency);
    for (int i = 0; ros::ok() && i != 200; ++i)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

TEST_CASE("encoder data", "[encoder data]")
{
    // this test case publishes encoder data and subscribes to
    // the turtle_interface node to check that it converts joint
    // states properly

    ros::NodeHandle n3;
    int frequency = 100;

    const auto pub3 = n3.advertise<nuturtlebot::SensorData>("/sensor_data", frequency, true);

    const auto sub3 = n3.subscribe("/joint_states", frequency, callback3);

    nuturtlebot::SensorData data;
    data.left_encoder = 2000;
    data.right_encoder = 1000;

    pub3.publish(data);

    ros::Rate loop_rate(frequency);
    for (int i = 0; ros::ok() && i != 200; ++i)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}