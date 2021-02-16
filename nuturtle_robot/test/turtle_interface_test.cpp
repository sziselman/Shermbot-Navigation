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

void wheelComCallbackTest1(const nuturtlebot::WheelCommands command)
{
    if (command.left_velocity != 0.0 || command.right_velocity != 0.0 || called1)
    {
        called1 = true;
        REQUIRE(command.left_velocity == 130);
        REQUIRE(command.right_velocity == 130);
    } else
    {
        REQUIRE(command.left_velocity == 0);
        REQUIRE(command.right_velocity == 0);
    }
}

void wheelComCallbackTest2(const nuturtlebot::WheelCommands command)
{
    if (command.left_velocity != 0.0 || command.right_velocity != 0.0 || called2)
    {
        called2 = true;
        REQUIRE(command.left_velocity == -104);
        REQUIRE(command.right_velocity == 104);
    } else 
    {
        REQUIRE(command.left_velocity == 0);
        REQUIRE(command.right_velocity == 0);
    }
}

void jointStateCallback(const sensor_msgs::JointState msg)
{
    using namespace rigid2d;

    if (msg.position[0] != 0.0 || msg.position[1] != 0.0 || called3)
    {
        called3 = true;
        CHECK(almost_equal(msg.position[0], 3.07));
        CHECK(almost_equal(msg.position[1], 1.53));
    } else
    {
        CHECK(msg.position[0] == 0.0);
        CHECK(msg.position[1] == 0.0);
    }
}

// TEST_CASE("pure translation", "[pure translation]")
// {
//     // this test case publishes a cmd_vel message and
//     // subscribes to the turtle_interface node to check 
//     // proper wheel commands
//     ROS_INFO("TEST 1");

//     ros::NodeHandle n;
//     int frequency = 100;

//     const auto cmdVel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", frequency, true);

//     const auto wheelCom_sub = n.subscribe("/wheel_cmd", frequency, wheelComCallbackTest1);

//     geometry_msgs::Twist twist_msg;
//     twist_msg.linear.x = 0.1;
//     twist_msg.linear.y = 0.0;
//     twist_msg.angular.z = 0.0;
//     ROS_INFO("linear x is %f\n", twist_msg.linear.x);

//     cmdVel_pub.publish(twist_msg);

//     ros::Rate loop_rate(frequency);
//     while (ros::ok())
//     {
//         ros::spinOnce();
//         cmdVel_pub.publish(twist_msg);
//         loop_rate.sleep();
//     }
// }

// TEST_CASE("pure rotation", "[pure rotation]")
// {
//     // this test case publishes a cmd_vel message and
//     // subscribes to the turtle_interface node to check
//     // proper wheel commands

//     ros::NodeHandle n;
//     int frequency = 100;

//     const auto cmdVel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", frequency, true);

//     const auto wheelCom_sub = n.subscribe("/wheel_cmd", frequency, wheelComCallbackTest2);

//     geometry_msgs::Twist twist_msg; 
//     twist_msg.linear.x = 0.0;
//     twist_msg.linear.y = 0.0;
//     twist_msg.angular.z = 0.1;

//     cmdVel_pub.publish(twist_msg);

//     ros::Rate loop_rate(frequency);
//     while (ros::ok())
//     {
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

// TEST_CASE("encoder data", "[encoder data]")
// {
//     // this test case publishes encoder data and subscribes to
//     // the turtle_interface node to check that it converts joint
//     // states properly

//     ros::NodeHandle n;
//     int frequency = 100;

//     const auto pub = n.advertise<nuturtlebot::SensorData>("/sensor_data", frequency, true);

//     const auto sub = n.subscribe("/joint_states", frequency, jointStateCallback);

//     nuturtlebot::SensorData data;
//     data.left_encoder = 2000;
//     data.left_encoder = 1000;

//     pub.publish(data);

//     ros::Rate loop_rate(frequency);
//     while (ros::ok())
//     {
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }
