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
///     visualization_msgs/MarkerArray (the ground truth markers)
///     visualization_msgs/MarkerArray (the fake sensor readings)
///     visualization_msgs/Marker (the walls)
///     nav_msgs/Path (the real path that the robot follows)
///     sensor_msgs/LaserScan (the lidar sensor messages)
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
    std::vector<double> t1_loc, t2_loc, t3_loc, t4_loc, t5_loc, t6_loc;
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

    n.getParam("tube1_location", t1_loc);
    n.getParam("tube2_location", t2_loc);
    n.getParam("tube3_location", t3_loc);
    n.getParam("tube4_location", t4_loc);
    n.getParam("tube5_location", t5_loc);
    n.getParam("tube6_location", t6_loc);

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
    std::vector<std::vector<double>> tube_locs{t1_loc, t2_loc, t3_loc, t4_loc, t5_loc, t6_loc};


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
        visualization_msgs::MarkerArray marker_array;

        tf2::Quaternion marker_quat;
        marker_quat.setRPY(0.0, 0.0, 0.0);
        geometry_msgs::Quaternion markerQuat = tf2::toMsg(marker_quat);

        for (int i = 0; i < tube_locs.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = world_frame_id;
            marker.header.stamp = current_time;
            marker.ns = "real";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = tube_locs[i][0];
            marker.pose.position.y = tube_locs[i][1];
            marker.pose.position.z = 0.1;
            marker.pose.orientation = markerQuat;
            marker.scale.x = tubeRad*2;
            marker.scale.y = tubeRad*2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 195. / 255.;
            marker.color.g = 205. / 255.;
            marker.color.b = 230. / 255.;
            marker.frame_locked = true;
            marker_array.markers.push_back(marker);
        }

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

        // marker_array.markers.push_back(wall);

        marker_true_pub.publish(marker_array);

        // if twist message has been received
        if (twist_received)
        {
            // Create the desired twist based on the message
            Twist2D desiredTwist;
            desiredTwist.dth = twist_msg.angular.z;
            desiredTwist.dx = twist_msg.linear.x;
            desiredTwist.dy = twist_msg.linear.y;

            // Add Gaussian noise to the commanded twist
            desiredTwist.dth += gaus_twist(get_random());
            desiredTwist.dx += gaus_twist(get_random());

            // Find wheel velocities required to achieve that twist
            wheelVel wheelVelocities = ninjaTurtle.convertTwist(desiredTwist);

            // Populate the sensor messages
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
            for (auto loc : tube_locs)
            {
                double distBetween = sqrt(pow(loc[0] - ninjaTurtle.getX(), 2) + pow(loc[1] - ninjaTurtle.getY(), 2));
                if (distBetween <= (tubeRad + robotRad))
                {
                    double dx = loc[0] - ninjaTurtle.getX();
                    double dy = loc[1] - ninjaTurtle.getY();
                    double dmag = sqrt(pow(dx, 2) + pow(dy, 2));

                    double move_x = dy / dmag;
                    double move_y = - dx / dmag;
                    
                    // have the robot move along that tangent line
                    // this is equivalent to the robot slipping
                    ninjaTurtle.changeConfig(move_x / 50, move_y / 50);
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

            visualization_msgs::MarkerArray marker_array_relative;

            for (int i = 0; i < tube_locs.size(); i++)
            {
                visualization_msgs::Marker marker_relative;
                marker_relative.header.frame_id = turtle_frame_id;
                marker_relative.header.stamp = current_time;
                marker_relative.ns = "relative";
                marker_relative.id = i;
                marker_relative.type = visualization_msgs::Marker::CYLINDER;
                
                std::vector<double> loc = tube_locs[i];

                // if the relative distance is out of range, then delete the marker from marker array
                double rel_distance = sqrt(pow(loc[0] - ninjaTurtle.getX(), 2) + pow(loc[1] - ninjaTurtle.getY(), 2));
                if (rel_distance > maxRange) {
                    marker_relative.action = visualization_msgs::Marker::DELETE;
                }
                // otherwise add it to the marker array
                else {
                    marker_relative.action = visualization_msgs::Marker::ADD;
                }

                // find coordinates of the tube relative to the turtle
                Vector2D tube_w(loc[0], loc[1]);
                Vector2D tube_t = T_tw(tube_w);

                marker_relative.pose.position.x = tube_t.x;
                marker_relative.pose.position.y = tube_t.y;
                marker_relative.pose.position.z = 0.1;
                marker_relative.pose.orientation = markerQuat;
                marker_relative.scale.x = tubeRad*2;
                marker_relative.scale.y = tubeRad*2;
                marker_relative.scale.z = 0.2;
                marker_relative.color.a = 1.0;
                marker_relative.color.r = 244. / 255.;
                marker_relative.color.g = 238. / 255.;
                marker_relative.color.b = 177. / 255.;
                marker_relative.frame_locked = true;

                marker_array_relative.markers.push_back(marker_relative);
            }

            marker_rel_pub.publish(marker_array_relative);

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

            // /*************
            //  * Publish simulated lidar scanner messages
            //  * **********/
            // std::vector<float> lidarRanges(360, maxRangeScan+1);
            // std::fill(lidarRanges.begin(),lidarRanges.end(),maxRangeScan+1);

            // for (auto marker : marker_array.markers)
            // {
            //     // angle of the tube relative to the world [-180, 180]
            //     int tubeAngle = round(rad2deg(atan2(marker.pose.position.y, marker.pose.position.x)));

            //     // shift the angle from [-180, 180] to [0, 359]
            //     if (tubeAngle < 0)
            //     {
            //         tubeAngle += 360;
            //     }

            //     // find (x1, y1), location of the turtle relative to the tube
            //     double x1 = ninjaTurtle.getX() - marker.pose.position.x;
            //     double y1 = ninjaTurtle.getY() - marker.pose.position.y;

            //     // look for points -20 and +20 degrees from the angle of the tube
            //     for (int i = tubeAngle - 20; i < tubeAngle + 20; ++i)
            //     {
            //         // find (x2, y2), based on the angle of the lidar scanner
            //         double x2 = x1 + maxRange * cos(deg2rad(i));
            //         double y2 = y1 + maxRange * sin(deg2rad(i));
                    
            //         double dx = x2 - x1;
            //         double dy = y2 - y1;
            //         double dr = sqrt(pow(dx, 2) + pow(dy, 2));
            //         double det = x1*y2 - x2*y1;
            //         double dis = pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2);

            //         double distance;
            //         // find the points of intersection

            //         if (fabs(dis) < 1e-5) // tangent
            //         {
            //             double intX = (det * dy) / pow(dr, 2);
            //             double intY = -(det * dx) / pow(dr, 2);
            //             distance = sqrt(pow(intX, 2) + pow(intY, 2));
            //         } else if (fabs(dis) > 0)
            //         {
            //             double intX1 = (det * dy + (dy / fabs(dy)) * dx * sqrt(pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2))) / pow(dr, 2);
            //             double intY1 = (-det * dx + fabs(dy) * sqrt(pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2))) / pow(dr, 2);
            //             double dist1 = sqrt(pow(intX1 - x1, 2) + pow(intY1 - y1, 2));

            //             double intX2 = (det * dy - (dy / fabs(dy)) * dx * sqrt(pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2))) / pow(dr, 2);
            //             double intY2 = (-det * dx - fabs(dy) * sqrt(pow(tubeRad, 2) * pow(dr, 2) - pow(det, 2))) / pow(dr, 2);
            //             double dist2 = sqrt(pow(intX2 - x1, 2) + pow(intY2 - y1, 2));

            //             if (dist1 < dist2)
            //             {
            //                 distance = dist1;
            //             } else
            //             {
            //                 distance = dist2;
            //             }
            //         }

            //         int index = i - int(rad2deg(ninjaTurtle.getTh()));
            //         index = index % 360;
            //         if (index < 0)
            //         {
            //             index += 360;
            //         }

            //         if (distance < lidarRanges[index])
            //         {
            //             lidarRanges[index] = distance;
            //         }
            //     }
            // }

            // /************
            //  * Check for Walls!!!!!!!!
            //  * *********/
            // for (int a = 0; a < 360; ++a)
            // {
            //     geometry_msgs::Point point1, point2;

            //     point1.x = ninjaTurtle.getX();
            //     point1.y = ninjaTurtle.getY();

            //     point2.x = point1.x + maxRange * cos(deg2rad(a));
            //     point2.y = point1.y + maxRange * sin(deg2rad(a));

            //     geometry_msgs::Point intercept;

            //     int wallIndex = round(a - int(rad2deg(ninjaTurtle.getTh())));
            //     wallIndex = wallIndex % 360;
            //     if (wallIndex < 0)
            //     {
            //         wallIndex += 360;
            //     } else if (wallIndex > 359)
            //     {
            //         wallIndex -= 360;
            //     }

            //     // check ground truth top line (upper left to upper right)
            //     geometry_msgs::Point inter1 = LineToLine(point1, point2, wall.points[0], wall.points[1]);
            //     double dist1 = sqrt(pow(inter1.x - point1.x, 2) + pow(inter1.y - point1.y, 2));
            //     if ((a < 180) && (dist1 < lidarRanges[wallIndex]))
            //     {
            //         lidarRanges[wallIndex] = dist1;
            //     }
            
            //     // check ground truth right line (upper right to lower right)
            //     geometry_msgs::Point inter2 = LineToLine(point1, point2, wall.points[1], wall.points[2]);
            //     double dist2 = sqrt(pow(inter2.x - point1.x, 2) + pow(inter2.y - point1.y, 2));
            //     if (((a < 90) || (a > 270)) && (dist2 < lidarRanges[wallIndex]))
            //     {
            //         lidarRanges[wallIndex] = dist2;
            //     }

            //     // check ground truth bottom line (lower right to lower left)
            //     geometry_msgs::Point inter3 = LineToLine(point1, point2, wall.points[2], wall.points[3]);
            //     double dist3 = sqrt(pow(inter3.x - point1.x, 2) + pow(inter3.y - point1.y, 2));
            //     if ((a > 180) && (dist3 < lidarRanges[wallIndex]))
            //     {
            //         lidarRanges[wallIndex] = dist3;
            //     }
                
            //     // check ground truth left line (lower left to upper left)
            //     geometry_msgs::Point inter4 = LineToLine(point1, point2, wall.points[3], wall.points[4]);
            //     double dist4 = sqrt(pow(inter4.x - point1.x, 2) + pow(inter4.y - point1.y, 2));
            //     if ((a > 90) && (a < 269) && (dist4 < lidarRanges[wallIndex]))
            //     {
            //         lidarRanges[wallIndex] = dist4;
            //     }
            // }

            // sensor_msgs::LaserScan scan_msg;
            // scan_msg.header.frame_id = turtle_frame_id;
            // scan_msg.header.stamp = current_time;
            // scan_msg.angle_min = 0;
            // scan_msg.angle_max = 2*PI;
            // scan_msg.angle_increment = PI / 180;
            // scan_msg.range_min = minRangeScan;
            // scan_msg.range_max = maxRangeScan;
            // scan_msg.ranges = lidarRanges;
            // scan_msg.intensities = std::vector<float> (360, 4000);
            
            // lidar_pub.publish(scan_msg);

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