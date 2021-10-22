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

class Landmarks {
    private:
        ros::NodeHandle n;
        ros::Subscriber lidar_sub;
        ros::Timer timer;
        bool scan_received = false;

    public:
        Landmarks() {
            lidar_sub = n.subscribe("/scan", 100, &Landmarks::scan_callback, this);
            timer = n.createTimer(ros::Duration(0.1), &Landmarks::main_loop, this);
        }

        void scan_callback(const sensor_msgs::LaserScan &msg) {
            scan_received = true;
            return;
        }
        void main_loop(const ros::TimerEvent &) {

        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "landmarks");
    Landmarks node;
    ros::spin();
    return 0;
}