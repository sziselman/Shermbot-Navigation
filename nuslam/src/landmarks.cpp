/// \file landmarks.cpp
/// \brief contains a node called landmarks to detect landmarks and publish their relative locations
/// 
/// PARAMETERS:     minRange (the minimum range that the Lidar sensor can sense)
///                 maxRange (the maximum range that the Lidar sensor can sesne)
///                 tubeRad (the radius of the tubes)
/// PUBLISHES:      visualization_msgs::MarkerArray (array of markers that are the detected landmarks)
/// SUBSCRIBES:     sensor_msgs::LaserScan (scan message from the Lidar Sensor)
/// SERVICES:       none

#include <ros/ros.h>

#include <nuslam/circle_fit_library.hpp>

#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>

#include <armadillo>
#include <vector>

/********
 * Helper Functions
 * *****/
void scanCallback(const sensor_msgs::LaserScan msg);

/********
 * Define global variables
 * *****/
sensor_msgs::LaserScan scan_msg;
static bool scan_received = false;

/********
 * Main Function
 * *****/
int main(int argc, char* argv[])
{
    using namespace arma;
    using namespace circle_fit;

    /********
     * Initialize the node & node handle
     * *****/
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle n;

    /********
     * Declare local variables
     * *****/
    int frequency = 100;
    double minRange, maxRange, tubeRad;

    ros::Rate loop_rate(frequency);

    /********
     * Read parameters from parameter server
     * *****/
    n.getParam("minimum_range", minRange);
    n.getParam("maximum_range", maxRange);
    n.getParam("tube_radius", tubeRad);

    /********
     * Define publisher, subscriber
     * *****/
    ros::Subscriber lidar_sub = n.subscribe("/scan", frequency, scanCallback);
    ros::Publisher landmark_pub = n.advertise<visualization_msgs::MarkerArray>("/real_sensor", frequency);

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    while (ros::ok())
    {
        current_time = ros::Time::now();
        ros::spinOnce();

        // if lidar scan message is received
        if (scan_received)
        {
            // cluster the points received from the lidar scan message
            std::vector<std::vector<geometry_msgs::Point>> clusters = ClusterPoints(scan_msg.ranges, minRange, maxRange);

            // create a marker array
            visualization_msgs::MarkerArray markerArray;

            int id = 0;

            // check each cluster to see if it is a circle
            for (auto cluster : clusters)
            {
                if (ClassifyCluster(cluster))
                {
                    // ROS_INFO_STREAM("yellow");
                    visualization_msgs::Marker marker = CircleFit(cluster);
                    if (marker.id < 0)
                    {
                        continue;
                    }

                    if (marker.scale.x > 2 * tubeRad)
                    {
                        continue;
                    }

                    marker.id = id;
                    id += 1;

                    markerArray.markers.push_back(marker);
                }
            }

            landmark_pub.publish(markerArray);

            scan_received = false;
        }

        last_time = current_time;
        loop_rate.sleep();
    }
    return 0;
}

void scanCallback(const sensor_msgs::LaserScan msg)
{
    scan_msg = msg;
    scan_received = true;
    return;
}