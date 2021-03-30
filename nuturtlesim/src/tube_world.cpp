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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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

geometry_msgs::Point LineToLine(geometry_msgs::Point point1, geometry_msgs::Point point2, geometry_msgs::Point point3, geometry_msgs::Point point4)
    {
        double x1 = point1.x;
        double y1 = point1.y;

        double x2 = point2.x;
        double y2 = point2.y;

        double x3 = point3.x;
        double y3 = point3.y;

        double x4 = point4.x;
        double y4 = point4.y;

        double det12 = x1*y2 - x2*y1;
        double det34 = x3*y4 - x4*y3;
        double det1234 = (x1-x2)*(y3-y4) - (x3-x4)*(y1-y2);

        geometry_msgs::Point intercept;
        intercept.x = ((det12*(x3-x4) - det34*(x1-x2))) / det1234;
        intercept.y = ((det12*(y3-y4) - det34*(y1-y2))) / det1234;

        return intercept;
    }

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

    double maxRangeScan, minRangeScan, scanRes, scanNoise;
    int angleIncr, sampleNum;

    double wallWidth, wallHeight;
    
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

    n.getParam("maximum_range", maxRangeScan);
    n.getParam("minimum_range", minRangeScan);
    n.getParam("angle_increment", angleIncr);
    n.getParam("sample_num", sampleNum);
    n.getParam("resolution", scanRes);
    n.getParam("noise_level", scanNoise);
    n.getParam("wall_width", wallWidth);
    n.getParam("wall_height", wallHeight);

    /***********
     * Initialize more local variables
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
    ros::Publisher wall_pub = n.advertise<visualization_msgs::Marker>("/wall", frequency);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/real_path", frequency);
    ros::Publisher lidar_pub = n.advertise<sensor_msgs::LaserScan>("/scan", frequency);

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
        marker1.scale.x = tubeRad*2;
        marker1.scale.y = tubeRad*2;
        marker1.scale.z = 0.2;
        marker1.color.a = 1.0;
        marker1.color.r = 250. / 255.;
        marker1.color.g = 192. / 255.;
        marker1.color.b = 221. / 255.;
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
        marker2.scale.x = tubeRad*2;
        marker2.scale.y = tubeRad*2;
        marker2.scale.z = 0.2;
        marker2.color.a = 1.0;
        marker2.color.r = 250. / 255.;
        marker2.color.g = 192. / 255.;
        marker2.color.b = 221. / 255.;
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
        marker3.scale.x = tubeRad*2;
        marker3.scale.y = tubeRad*2;
        marker3.scale.z = 0.2;
        marker3.color.a = 1.0;
        marker3.color.r = 250. / 255.;
        marker3.color.g = 192. / 255.;
        marker3.color.b = 221. / 255.;
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
        marker4.scale.x = tubeRad*2;
        marker4.scale.y = tubeRad*2;
        marker4.scale.z = 0.2;
        marker4.color.a = 1.0;
        marker4.color.r = 250. / 255.;
        marker4.color.g = 192. / 255.;
        marker4.color.b = 221. / 255.;
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
        marker5.scale.x = tubeRad*2;
        marker5.scale.y = tubeRad*2;
        marker5.scale.z = 0.2;
        marker5.color.a = 1.0;
        marker5.color.r = 250. / 255.;
        marker5.color.g = 192. / 255.;
        marker5.color.b = 221. / 255.;
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
        marker6.scale.x = tubeRad*2;
        marker6.scale.y = tubeRad*2;
        marker6.scale.z = 0.2;
        marker6.color.a = 1.0;
        marker6.color.r = 250. / 255.;
        marker6.color.g = 192. / 255.;
        marker6.color.b = 221. / 255.;
        marker6.frame_locked = true;

        markerArray.markers.push_back(marker6);

        // walls
        visualization_msgs::Marker wall;
        geometry_msgs::Point upLeft, upRight, loLeft, loRight;

        upLeft.x = -wallWidth/2;
        upLeft.y = wallHeight/2;

        upRight.x = wallWidth/2;
        upRight.y = wallHeight/2;

        loLeft.x = -wallWidth/2;
        loLeft.y = -wallHeight/2;

        loRight.x = wallWidth/2;
        loRight.y = -wallHeight/2;

        wall.header.frame_id = world_frame_id;
        wall.header.stamp = current_time;
        wall.ns = "real";
        wall.type = 4;
        wall.action = visualization_msgs::Marker::ADD;
        wall.points.push_back(upLeft);
        wall.points.push_back(upRight);
        wall.points.push_back(loRight);
        wall.points.push_back(loLeft);
        wall.points.push_back(upLeft);
        wall.scale.x = 0.01;
        wall.color.a = 1;
        wall.color.r = 250. / 255.;
        wall.color.g = 192. / 255.;
        wall.color.b = 221. / 255.;
        
        wall_pub.publish(wall);

        // markerArray.markers.push_back(wall);

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
            for (auto loc : listOfTubes)
            {
                double distBetween = sqrt(pow(loc[0] - ninjaTurtle.getX(), 2) + pow(loc[1] - ninjaTurtle.getY(), 2));
                if (distBetween <= (tubeRad + robotRad))
                {
                    double dx = (loc[0] - ninjaTurtle.getX())/20;
                    double dy = (loc[1] - ninjaTurtle.getY())/20;
                    
                    // // have the robot move along that tangent line
                    ninjaTurtle.changeConfig(dy, dx);
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
            markerRel1.scale.x = tubeRad*2;
            markerRel1.scale.y = tubeRad*2;
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
            markerRel2.scale.x = tubeRad*2;
            markerRel2.scale.y = tubeRad*2;
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
            markerRel3.scale.x = tubeRad*2;
            markerRel3.scale.y = tubeRad*2;
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
            markerRel4.scale.x = tubeRad*2;
            markerRel4.scale.y = tubeRad*2;
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
            markerRel5.scale.x = tubeRad*2;
            markerRel5.scale.y = tubeRad*2;
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
            markerRel6.scale.x = tubeRad*2;
            markerRel6.scale.y = tubeRad*2;
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

            /*************
             * Publish simulated lidar scanner messages
             * **********/
            std::vector<float> lidarRanges(360, maxRangeScan+1);
            std::fill(lidarRanges.begin(),lidarRanges.end(),maxRangeScan+1);

            for (auto marker : markerArray.markers)
            {
                // angle of the tube relative to the world [-180, 180]
                int tubeAngle = round(rad2deg(atan2(marker.pose.position.y, marker.pose.position.x)));

                // shift the angle from [-180, 180] to [0, 359]
                if (tubeAngle < 0)
                {
                    tubeAngle += 360;
                }

                // find (x1, y1), location of the turtle relative to the tube
                double x1 = ninjaTurtle.getX() - marker.pose.position.x;
                double y1 = ninjaTurtle.getY() - marker.pose.position.y;

                // look for points -20 and +20 degrees from the angle of the tube
                for (int i = tubeAngle - 20; i < tubeAngle + 20; ++i)
                {
                    // find (x2, y2), based on the angle of the lidar scanner
                    double x2 = x1 + maxRange * cos(deg2rad(i));
                    double y2 = y1 + maxRange * sin(deg2rad(i));
                    
                    double dx = x2 - x1;
                    double dy = y2 - y1;
                    double dr = sqrt(pow(dx, 2) + pow(dy, 2));
                    double det = x1*y2 - x2*y1;
                    double dis = pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2);

                    double distance;
                    // find the points of intersection

                    if (fabs(dis) < 1e-5) // tangent
                    {
                        double intX = (det * dy) / pow(dr, 2);
                        double intY = -(det * dx) / pow(dr, 2);
                        distance = sqrt(pow(intX, 2) + pow(intY, 2));
                    } else if (fabs(dis) > 0)
                    {
                        double intX1 = (det * dy + (dy / fabs(dy)) * dx * sqrt(pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2))) / pow(dr, 2);
                        double intY1 = (-det * dx + fabs(dy) * sqrt(pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2))) / pow(dr, 2);
                        double dist1 = sqrt(pow(intX1 - x1, 2) + pow(intY1 - y1, 2));

                        double intX2 = (det * dy - (dy / fabs(dy)) * dx * sqrt(pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2))) / pow(dr, 2);
                        double intY2 = (-det * dx - fabs(dy) * sqrt(pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2))) / pow(dr, 2);
                        double dist2 = sqrt(pow(intX2 - x1, 2) + pow(intY2 - y1, 2));

                        if (dist1 < dist2)
                        {
                            distance = dist1;
                        } else
                        {
                            distance = dist2;
                        }
                    }

                    int index = i - int(rad2deg(ninjaTurtle.getTh()));
                    index = index % 360;
                    if (index < 0)
                    {
                        index += 360;
                    }

                    if (distance < lidarRanges[index])
                    {
                        lidarRanges[index] = distance;
                    }
                }
            }

            /************
             * Check for Walls!!!!!!!!
             * *********/
            for (int a = 0; a < 360; ++a)
            {
                geometry_msgs::Point point1, point2;

                point1.x = ninjaTurtle.getX();
                point1.y = ninjaTurtle.getY();

                point2.x = point1.x + maxRange * cos(deg2rad(a));
                point2.y = point1.y + maxRange * sin(deg2rad(a));

                geometry_msgs::Point intercept;

                int wallIndex = round(a - int(rad2deg(ninjaTurtle.getTh())));
                wallIndex = wallIndex % 360;
                if (wallIndex < 0)
                {
                    wallIndex += 360;
                } else if (wallIndex > 359)
                {
                    wallIndex -= 360;
                }

                // check ground truth top line (upper left to upper right)
                geometry_msgs::Point inter1 = LineToLine(point1, point2, wall.points[0], wall.points[1]);
                double dist1 = sqrt(pow(inter1.x - point1.x, 2) + pow(inter1.y - point1.y, 2));
                if ((a < 180) && (dist1 < lidarRanges[wallIndex]))
                {
                    lidarRanges[wallIndex] = dist1;
                }
            
                // check ground truth right line (upper right to lower right)
                geometry_msgs::Point inter2 = LineToLine(point1, point2, wall.points[1], wall.points[2]);
                double dist2 = sqrt(pow(inter2.x - point1.x, 2) + pow(inter2.y - point1.y, 2));
                if (((a < 90) || (a > 270)) && (dist2 < lidarRanges[wallIndex]))
                {
                    lidarRanges[wallIndex] = dist2;
                }

                // check ground truth bottom line (lower right to lower left)
                geometry_msgs::Point inter3 = LineToLine(point1, point2, wall.points[2], wall.points[3]);
                double dist3 = sqrt(pow(inter3.x - point1.x, 2) + pow(inter3.y - point1.y, 2));
                if ((a > 180) && (dist3 < lidarRanges[wallIndex]))
                {
                    lidarRanges[wallIndex] = dist3;
                }
                
                // check ground truth left line (lower left to upper left)
                geometry_msgs::Point inter4 = LineToLine(point1, point2, wall.points[3], wall.points[4]);
                double dist4 = sqrt(pow(inter4.x - point1.x, 2) + pow(inter4.y - point1.y, 2));
                if ((a > 90) && (a < 269) && (dist4 < lidarRanges[wallIndex]))
                {
                    lidarRanges[wallIndex] = dist4;
                }
            }

            sensor_msgs::LaserScan scan_msg;
            scan_msg.header.frame_id = turtle_frame_id;
            scan_msg.header.stamp = current_time;
            scan_msg.angle_min = 0;
            scan_msg.angle_max = 2*PI;
            scan_msg.angle_increment = PI / 180;
            scan_msg.range_min = minRangeScan;
            scan_msg.range_max = maxRangeScan;
            scan_msg.ranges = lidarRanges;
            scan_msg.intensities = std::vector<float> (360, 4000);
            
            lidar_pub.publish(scan_msg);

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