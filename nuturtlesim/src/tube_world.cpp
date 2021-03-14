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
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <random>
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>

/***********
 * Declare global variables
 * ********/
static geometry_msgs::Twist twist_msg;
static bool twist_received = false;

/***********
 * Helper Functions
 * ********/
void twistCallback(const geometry_msgs::Twist msg);

/***********
 * get_random() function
 * ********/
std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

/***********
 * Main Function
 * ********/
int main(int argc, char* argv[])
{
    using namespace rigid2d;

    /***********
     * Initialize the node & node handle
     * ********/
    ros::init(argc, argv, "tube_world");
    ros::NodeHandle n;

    /***********
     * Initialize local variables
     * ********/
    int frequency = 10;
    double tubeRad, wheelRad, wheelBase, maxRange, twistNoise, slipMin, slipMax, robotRad;
    bool latch = true;
    
    std::string world_frame_id, turtle_frame_id, left_wheel_joint, right_wheel_joint;
    std::string odom_frame_id;
    std::vector<double> tube1_loc, tube2_loc, tube3_loc, tube4_loc, tube5_loc, tube6_loc;

    nav_msgs::Path path;

    /***********
     * Read parameters from parameter server
     * ********/
    n.getParam("max_range", maxRange);
    n.getParam("wheel_base", wheelBase);
    n.getParam("wheel_radius", wheelRad);
    n.getParam("odom_frame_id", odom_frame_id);
    n.getParam("left_wheel_joint", left_wheel_joint);
    n.getParam("right_wheel_joint", right_wheel_joint);
    n.getParam("tube1_location", tube1_loc);
    n.getParam("tube2_location", tube2_loc);
    n.getParam("tube3_location", tube3_loc);
    n.getParam("tube4_location", tube4_loc);
    n.getParam("tube5_location", tube5_loc);
    n.getParam("tube6_location", tube6_loc);
    n.getParam("tube_radius", tubeRad);
    n.getParam("world_frame_id", world_frame_id);
    n.getParam("turtle_frame_id", turtle_frame_id);
    n.getParam("twist_noise", twistNoise);
    n.getParam("slip_min", slipMin);
    n.getParam("slip_max", slipMax);
    n.getParam("robot_radius", robotRad);

    /***********
     * Initialize mroe local variables
     * ********/
    std::normal_distribution<> gaus_twist(0, twistNoise);

    double slipMean = (slipMin + slipMax) / 2;
    double slipVar = slipMax - slipMean;

    std::normal_distribution<> slip_noise(slipMean, slipVar);
    std::list<std::vector<double>> listOfTubes({tube1_loc, tube2_loc, tube3_loc, tube4_loc, tube5_loc, tube6_loc});


    /***********
     * Define publisher, subscriber, service and clients
     * ********/
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", frequency);
    ros::Publisher marker_true_pub = n.advertise<visualization_msgs::MarkerArray>("/ground_truth", frequency, latch);
    ros::Publisher marker_rel_pub = n.advertise<visualization_msgs::MarkerArray>("/fake_sensor", frequency);
    // ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", frequency);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/real_path", frequency);

    ros::Subscriber twist_sub = n.subscribe("/cmd_vel", frequency, twistCallback);

    ros::Rate loop_rate(frequency);

    tf2_ros::TransformBroadcaster broadcaster;

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    /***********
     * Initialize joint states message
     * ********/
    DiffDrive ninjaTurtle = DiffDrive(wheelBase, wheelRad, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    sensor_msgs::JointState joint_msg;

    joint_msg.name.push_back(left_wheel_joint);
    joint_msg.name.push_back(right_wheel_joint);

    joint_msg.position.push_back(0.0);
    joint_msg.position.push_back(0.0);
    
    joint_pub.publish(joint_msg);

    while(ros::ok())
    {
        current_time = ros::Time::now();
        ros::spinOnce();

        /************
         * Publish cylindrical markers (GROUND TRUTH)
         * *********/
        visualization_msgs::MarkerArray markerArray;

        tf2::Quaternion marker_quat;
        marker_quat.setRPY(0.0, 0.0, 0.0);
        geometry_msgs::Quaternion markerQuat = tf2::toMsg(marker_quat);

        // marker1
        visualization_msgs::Marker marker1;
        marker1.header.frame_id = world_frame_id;
        marker1.header.stamp = current_time;
        marker1.ns = "real";
        marker1.id = 0;
        marker1.type = visualization_msgs::Marker::CYLINDER;
        marker1.action = visualization_msgs::Marker::ADD;
        
        marker1.pose.position.x = tube1_loc[0];
        marker1.pose.position.y = tube1_loc[1];
        marker1.pose.position.z = 0.1;
        marker1.pose.orientation = markerQuat;
        marker1.scale.x = tubeRad;
        marker1.scale.y = tubeRad;
        marker1.scale.z = 0.2;
        marker1.color.a = 1.0;
        marker1.color.r = 1.0;
        marker1.color.g = 1.0;
        marker1.color.b = 1.0;
        marker1.frame_locked = true;

        markerArray.markers.push_back(marker1);

        // marker2
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = world_frame_id;
        marker2.header.stamp = current_time;
        marker2.ns = "real";
        marker2.id = 1;
        marker2.type = visualization_msgs::Marker::CYLINDER;
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.pose.position.x = tube2_loc[0];
        marker2.pose.position.y = tube2_loc[1];
        marker2.pose.position.z = 0.1;
        marker2.pose.orientation = markerQuat;
        marker2.scale.x = tubeRad;
        marker2.scale.y = tubeRad;
        marker2.scale.z = 0.2;
        marker2.color.a = 1.0;
        marker2.color.r = 1.0;
        marker2.color.g = 1.0;
        marker2.color.b = 1.0;
        marker2.frame_locked = true;

        markerArray.markers.push_back(marker2);

        // marker3
        visualization_msgs::Marker marker3;
        marker3.header.frame_id = world_frame_id;
        marker3.header.stamp = current_time;
        marker3.ns = "real";
        marker3.id = 2;
        marker3.type = visualization_msgs::Marker::CYLINDER;
        marker3.action = visualization_msgs::Marker::ADD;
        marker3.pose.position.x = tube3_loc[0];
        marker3.pose.position.y = tube3_loc[1];
        marker3.pose.position.z = 0.1;
        marker3.pose.orientation = markerQuat;
        marker3.scale.x = tubeRad;
        marker3.scale.y = tubeRad;
        marker3.scale.z = 0.2;
        marker3.color.a = 1.0;
        marker3.color.r = 1.0;
        marker3.color.g = 1.0;
        marker3.color.b = 1.0;
        marker3.frame_locked = true;

        markerArray.markers.push_back(marker3);

        // marker4
        visualization_msgs::Marker marker4;
        marker4.header.frame_id = world_frame_id;
        marker4.header.stamp = current_time;
        marker4.ns = "real";
        marker4.id = 3;
        marker4.type = visualization_msgs::Marker::CYLINDER;
        marker4.action = visualization_msgs::Marker::ADD;
        marker4.pose.position.x = tube4_loc[0];
        marker4.pose.position.y = tube4_loc[1];
        marker4.pose.position.z = 0.1;
        marker4.pose.orientation = markerQuat;
        marker4.scale.x = tubeRad;
        marker4.scale.y = tubeRad;
        marker4.scale.z = 0.2;
        marker4.color.a = 1.0;
        marker4.color.r = 1.0;
        marker4.color.g = 1.0;
        marker4.color.b = 1.0;
        marker4.frame_locked = true;

        markerArray.markers.push_back(marker4);

        // marker5
        visualization_msgs::Marker marker5;
        marker5.header.frame_id = world_frame_id;
        marker5.header.stamp = current_time;
        marker5.ns = "real";
        marker5.id = 4;
        marker5.type = visualization_msgs::Marker::CYLINDER;
        marker5.action = visualization_msgs::Marker::ADD;
        marker5.pose.position.x = tube5_loc[0];
        marker5.pose.position.y = tube5_loc[1];
        marker5.pose.position.z = 0.1;
        marker5.pose.orientation = markerQuat;
        marker5.scale.x = tubeRad;
        marker5.scale.y = tubeRad;
        marker5.scale.z = 0.2;
        marker5.color.a = 1.0;
        marker5.color.r = 1.0;
        marker5.color.g = 1.0;
        marker5.color.b = 1.0;
        marker5.frame_locked = true;

        markerArray.markers.push_back(marker5);

        // marker6
        visualization_msgs::Marker marker6;
        marker6.header.frame_id = world_frame_id;
        marker6.header.stamp = current_time;
        marker6.ns = "real";
        marker6.id = 5;
        marker6.type = visualization_msgs::Marker::CYLINDER;
        marker6.action = visualization_msgs::Marker::ADD;
        marker6.pose.position.x = tube6_loc[0];
        marker6.pose.position.y = tube6_loc[1];
        marker6.pose.position.z = 0.1;
        marker6.pose.orientation = markerQuat;
        marker6.scale.x = tubeRad;
        marker6.scale.y = tubeRad;
        marker6.scale.z = 0.2;
        marker6.color.a = 1.0;
        marker6.color.r = 1.0;
        marker6.color.g = 1.0;
        marker6.color.b = 1.0;
        marker6.frame_locked = true;

        markerArray.markers.push_back(marker6);

        marker_true_pub.publish(markerArray);



        // if twist message has been received
        if (twist_received)
        {
            /**************
             * Create the desired twist based on the message
             * ***********/
            Twist2D desiredTwist;
            desiredTwist.dth = twist_msg.angular.z;
            desiredTwist.dx = twist_msg.linear.x;
            desiredTwist.dy = twist_msg.linear.y;

            /*************
             * Add Gaussian noise to the commanded twist
             * **********/
            desiredTwist.dth += gaus_twist(get_random());
            desiredTwist.dx += gaus_twist(get_random());

            /*************
             * Find wheel velocities required to achieve that twist
             * **********/
            wheelVel wheelVelocities = ninjaTurtle.convertTwist(desiredTwist);

            /*************
             * Populate the sensor messages
             * **********/
            joint_msg.header.stamp = current_time;
            joint_msg.header.frame_id = turtle_frame_id;

            joint_msg.position[0] += wheelVelocities.uL * (current_time - last_time).toSec();
            joint_msg.position[1] += wheelVelocities.uR * (current_time - last_time).toSec();

            /***********
             * Add wheel slip noise using slip model nu*omega where nu is uniform random noise between
             * slipMin and slipMax
             * ********/
            joint_msg.position[0] += wheelVelocities.uL * slip_noise(get_random());
            joint_msg.position[1] += wheelVelocities.uR * slip_noise(get_random());

            /************
             * Update configuration of diff-drive robot based on new wheel angles
             * Publish joint_state message
             * *********/
            ninjaTurtle(joint_msg.position[0], joint_msg.position[1]);

            joint_pub.publish(joint_msg);

            /***********
             * COLLISION DETECTION
             * ********/

            // find the tube th
            for (auto loc : listOfTubes)
            {
                double distBetween = sqrt(pow(loc[0] - ninjaTurtle.getX(), 2) + pow(loc[1] - ninjaTurtle.getY(), 2));
                if (distBetween <= (tubeRad + robotRad))
                {
                    // find the tangent line between the tube circle and the robot circle
                    // have the robot move along that tangent line
                }
            }

            /***********
             * Publish a transform between world frame and turtle frame to indicate location of robot
             * ********/
            tf2::Quaternion odom_quater;
            odom_quater.setRPY(0, 0, ninjaTurtle.getTh());

            geometry_msgs::Quaternion odom_quat = tf2::toMsg(odom_quater);

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = world_frame_id;
            odom_trans.child_frame_id = turtle_frame_id;

            odom_trans.transform.translation.x = ninjaTurtle.getX();
            odom_trans.transform.translation.y = ninjaTurtle.getY();
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;
        
            broadcaster.sendTransform(odom_trans);

            // nav_msgs::Odometry odom;
            // odom.header.stamp = current_time;
            // odom.header.frame_id = "odom";

            // odom.pose.pose.position.x = ninjaTurtle.getX();
            // odom.pose.pose.position.y = ninjaTurtle.getY();
            // odom.pose.pose.position.z = 0.0;
            // odom.pose.pose.orientation = odom_quat;

            // odom.child_frame_id = turtle_frame_id;
            // odom.twist.twist.linear.x = desiredTwist.dx;
            // odom.twist.twist.linear.y = desiredTwist.dy;
            // odom.twist.twist.angular.z = desiredTwist.dth;

            // odom_pub.publish(odom);

            /***********
             * FAKE_SENSOR markers
             * Publish cylindrical markers relative to the location of the robot
             * ********/

            // find the transformation between the world frame and the turtle frame
            Vector2D transRobot(ninjaTurtle.getX(), ninjaTurtle.getY());
            Transform2D T_wt = Transform2D(transRobot, ninjaTurtle.getTh());
            Transform2D T_tw = T_wt.inv();

            visualization_msgs::MarkerArray markerArrayRel;

            // relative marker for tube 1
            visualization_msgs::Marker markerRel1;
            markerRel1.header.frame_id = turtle_frame_id;
            markerRel1.header.stamp = current_time;
            markerRel1.ns = "relative";
            markerRel1.id = 0;
            markerRel1.type = visualization_msgs::Marker::CYLINDER;
            if (sqrt(pow(tube1_loc[0] - ninjaTurtle.getX(), 2) + pow(tube1_loc[1] - ninjaTurtle.getY(), 2)) > maxRange)
            {
                markerRel1.action = visualization_msgs::Marker::DELETE;
            } else
            {
                markerRel1.action = visualization_msgs::Marker::ADD;
            }

            // find the coordinates of tube 1 relative to the turtle
            Vector2D tube1_w(tube1_loc[0], tube1_loc[1]);
            Vector2D tube1_t = T_tw(tube1_w);

            markerRel1.pose.position.x = tube1_t.x;
            markerRel1.pose.position.y = tube1_t.y;
            markerRel1.pose.position.z = 0.1;
            markerRel1.pose.orientation = markerQuat;
            markerRel1.scale.x = tubeRad;
            markerRel1.scale.y = tubeRad;
            markerRel1.scale.z = 0.2;
            markerRel1.color.a = 1.0;
            markerRel1.color.r = 1.0;
            markerRel1.color.g = 1.0;
            markerRel1.color.b = 1.0;
            markerRel1.frame_locked = true;

            markerArrayRel.markers.push_back(markerRel1);

            // relative marker for tube 2
            visualization_msgs::Marker markerRel2;
            markerRel2.header.frame_id = turtle_frame_id;
            markerRel2.header.stamp = current_time;
            markerRel2.ns = "relative";
            markerRel2.id = 1;
            markerRel2.type = visualization_msgs::Marker::CYLINDER;
            if (sqrt(pow(tube2_loc[0] - ninjaTurtle.getX(), 2) + pow(tube2_loc[1] - ninjaTurtle.getY(), 2)) > maxRange)
            {
                markerRel2.action = visualization_msgs::Marker::DELETE;
            } else
            {
                markerRel2.action = visualization_msgs::Marker::ADD;
            }

            // find the coordinates of tube 2 relative to the turtle
            Vector2D tube2_w(tube2_loc[0], tube2_loc[1]);
            Vector2D tube2_t = T_tw(tube2_w);

            markerRel2.pose.position.x = tube2_t.x;
            markerRel2.pose.position.y = tube2_t.y;
            markerRel2.pose.position.z = 0.1;
            markerRel2.pose.orientation = markerQuat;
            markerRel2.scale.x = tubeRad;
            markerRel2.scale.y = tubeRad;
            markerRel2.scale.z = 0.2;
            markerRel2.color.a = 1.0;
            markerRel2.color.r = 1.0;
            markerRel2.color.g = 1.0;
            markerRel2.color.b = 1.0;
            markerRel2.frame_locked = true;

            markerArrayRel.markers.push_back(markerRel2);

            // relative marker for tube 3
            visualization_msgs::Marker markerRel3;
            markerRel3.header.frame_id = turtle_frame_id;
            markerRel3.header.stamp = current_time;
            markerRel3.ns = "relative";
            markerRel3.id = 2;
            markerRel3.type = visualization_msgs::Marker::CYLINDER;
            if (sqrt(pow(tube3_loc[0] - ninjaTurtle.getX(), 2) + pow(tube3_loc[1] - ninjaTurtle.getY(), 2)) > maxRange)
            {
                markerRel3.action = visualization_msgs::Marker::DELETE;
            } else
            {
                markerRel3.action = visualization_msgs::Marker::ADD;
            }

            // find the coordinates of tube 3 relative to the turtle
            Vector2D tube3_w(tube3_loc[0], tube3_loc[1]);
            Vector2D tube3_t = T_tw(tube3_w);

            markerRel3.pose.position.x = tube3_t.x;
            markerRel3.pose.position.y = tube3_t.y;
            markerRel3.pose.position.z = 0.1;
            markerRel3.pose.orientation = markerQuat;
            markerRel3.scale.x = tubeRad;
            markerRel3.scale.y = tubeRad;
            markerRel3.scale.z = 0.2;
            markerRel3.color.a = 1.0;
            markerRel3.color.r = 1.0;
            markerRel3.color.g = 1.0;
            markerRel3.color.b = 1.0;
            markerRel3.frame_locked = true;

            markerArrayRel.markers.push_back(markerRel3);

            // relative marker for tube 4
            visualization_msgs::Marker markerRel4;
            markerRel4.header.frame_id = turtle_frame_id;
            markerRel4.header.stamp = current_time;
            markerRel4.ns = "relative";
            markerRel4.id = 3;
            markerRel4.type = visualization_msgs::Marker::CYLINDER;
            if (sqrt(pow(tube4_loc[0] - ninjaTurtle.getX(), 2) + pow(tube4_loc[1] - ninjaTurtle.getY(), 2)) > maxRange)
            {
                markerRel4.action = visualization_msgs::Marker::DELETE;
            } else
            {
                markerRel4.action = visualization_msgs::Marker::ADD;
            }

            // find the coordinates of tube 4 relative to the turtle
            Vector2D tube4_w(tube4_loc[0], tube4_loc[1]);
            Vector2D tube4_t = T_tw(tube4_w);

            markerRel4.pose.position.x = tube4_t.x;
            markerRel4.pose.position.y = tube4_t.y;
            markerRel4.pose.position.z = 0.1;
            markerRel4.pose.orientation = markerQuat;
            markerRel4.scale.x = tubeRad;
            markerRel4.scale.y = tubeRad;
            markerRel4.scale.z = 0.2;
            markerRel4.color.a = 1.0;
            markerRel4.color.r = 1.0;
            markerRel4.color.g = 1.0;
            markerRel4.color.b = 1.0;
            markerRel4.frame_locked = true;

            markerArrayRel.markers.push_back(markerRel4);
            
            // relative marker for tube 5
            visualization_msgs::Marker markerRel5;
            markerRel5.header.frame_id = turtle_frame_id;
            markerRel5.header.stamp = current_time;
            markerRel5.ns = "relative";
            markerRel5.id = 4;
            markerRel5.type = visualization_msgs::Marker::CYLINDER;
            if (sqrt(pow(tube5_loc[0] - ninjaTurtle.getX(), 2) + pow(tube5_loc[1] - ninjaTurtle.getY(), 2)) > maxRange)
            {
                markerRel5.action = visualization_msgs::Marker::DELETE;
            } else
            {
                markerRel5.action = visualization_msgs::Marker::ADD;
            }

            // find the coordinates of tube 1 relative to the turtle
            Vector2D tube5_w(tube5_loc[0], tube5_loc[1]);
            Vector2D tube5_t = T_tw(tube5_w);


            markerRel5.pose.position.x = tube5_t.x;
            markerRel5.pose.position.y = tube5_t.y;
            markerRel5.pose.position.z = 0.1;
            markerRel5.pose.orientation = markerQuat;
            markerRel5.scale.x = tubeRad;
            markerRel5.scale.y = tubeRad;
            markerRel5.scale.z = 0.2;
            markerRel5.color.a = 1.0;
            markerRel5.color.r = 1.0;
            markerRel5.color.g = 1.0;
            markerRel5.color.b = 1.0;
            markerRel5.frame_locked = true;

            markerArrayRel.markers.push_back(markerRel5);

            // relative marker for tube 6
            visualization_msgs::Marker markerRel6;
            markerRel6.header.frame_id = turtle_frame_id;
            markerRel6.header.stamp = current_time;
            markerRel6.ns = "relative";
            markerRel6.id = 5;
            markerRel6.type = visualization_msgs::Marker::CYLINDER;
            if (sqrt(pow(tube6_loc[0] - ninjaTurtle.getX(), 2) + pow(tube6_loc[1] - ninjaTurtle.getY(), 2)) > maxRange)
            {
                markerRel6.action = visualization_msgs::Marker::DELETE;
            } else
            {
                markerRel6.action = visualization_msgs::Marker::ADD;
            }

            // find the coordinates of tube 6 relative to the turtle
            Vector2D tube6_w(tube6_loc[0], tube6_loc[1]);
            Vector2D tube6_t = T_tw(tube6_w);

            markerRel6.pose.position.x = tube6_t.x;
            markerRel6.pose.position.y = tube6_t.y;
            markerRel6.pose.position.z = 0.1;
            markerRel6.pose.orientation = markerQuat;
            markerRel6.scale.x = tubeRad;
            markerRel6.scale.y = tubeRad;
            markerRel6.scale.z = 0.2;
            markerRel6.color.a = 1.0;
            markerRel6.color.r = 1.0;
            markerRel6.color.g = 1.0;
            markerRel6.color.b = 1.0;
            markerRel6.frame_locked = true;

            markerArrayRel.markers.push_back(markerRel6);

            marker_rel_pub.publish(markerArrayRel);

            /**************
             * Publish a message to show the actual robot trajectory
             * ***********/
            geometry_msgs::PoseStamped poseStamp;
            path.header.stamp = current_time;
            path.header.frame_id = world_frame_id;
            poseStamp.pose.position.x = ninjaTurtle.getX();
            poseStamp.pose.position.y = ninjaTurtle.getY();
            poseStamp.pose.orientation.z = ninjaTurtle.getTh();

            path.poses.push_back(poseStamp);
            path_pub.publish(path);
            twist_received = false;
        }

    last_time = current_time;
    loop_rate.sleep();
    }

    return 0;
}

void twistCallback(const geometry_msgs::Twist msg)
{
    twist_msg = msg;
    twist_received = true;
    return;
}