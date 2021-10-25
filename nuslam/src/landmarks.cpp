/// \file landmarks.cpp
/// \brief contains a node called landmarks to detect landmarks and publish their relative locations
/// 
/// PARAMETERS:     min_range (the minimum range that the Lidar sensor can sense)
///                 max_range (the maximum range that the Lidar sensor can sesne)
///                 tube_rad (the radius of the tubes)
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
#include <string>

class Landmarks {
    private:
        // ros variables
        ros::NodeHandle n;
        ros::Subscriber lidar_sub;
        ros::Publisher landmark_pub;
        // ros::Timer timer;
        bool scan_received = false;
        std::vector<std::vector<geometry_msgs::Point>> clusters;
        visualization_msgs::MarkerArray marker_array;

        // variables from parameter server
        double min_range, max_range;
        double tube_rad;
        std::string turtle_frame_id;
        int frequency = 10;

    public:
        Landmarks() {
            load_parameters();

            lidar_sub = n.subscribe("/scan", 100, &Landmarks::scan_callback, this);
            landmark_pub = n.advertise<visualization_msgs::MarkerArray>("/real_sensor", 10);
            // timer = n.createTimer(ros::Duration(0.1), &Landmarks::main_loop, this);
        }

        void load_parameters(void) {
            n.getParam("minimum_range", min_range);
            n.getParam("maximum_range", max_range);
            n.getParam("tube_radius", tube_rad);
            n.getParam("turtle_frame_id", turtle_frame_id);

            return;
        }

        void scan_callback(const sensor_msgs::LaserScan &msg) {
            using namespace circle_fit;
            scan_received = true;
            clusters = clusterPoints(msg.ranges, min_range, max_range);

            return;
        }
        
        void main_loop(void) {
            using namespace circle_fit;

            ros::Rate loop_rate(frequency);

            while (ros::ok()) {
                
                if (scan_received) {
                    
                    int id = 0;

                    marker_array.markers.clear();

                    // std::cout << "new marker array\r" << std::endl;

                    // check to see if each cluster is a circle
                    for (auto cluster : clusters) {

                        if (classifyCluster(cluster)) {
                            // std::cout << "cluster is a circle!!\r" << std::endl;

                            visualization_msgs::Marker marker = circleFit(cluster);

                            if (marker.id < 0) {
                                continue;
                            }

                            if (marker.scale.x/2 > 1) {
                                continue;
                            }

                            // std::cout << "(x: " << marker.pose.position.x << ", y: " << marker.pose.position.y << ")\r" << std::endl;
                            marker.header.stamp = ros::Time::now();
                            marker.header.frame_id = turtle_frame_id;
                            marker.scale.x = 2*tube_rad;
                            marker.scale.y = 2*tube_rad;
                            marker.id = id; 
                            id++;

                            marker_array.markers.push_back(marker);
                        }
                    }

                    landmark_pub.publish(marker_array);

                    scan_received = false;
                }
                loop_rate.sleep();
                ros::spinOnce();
            }
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "landmarks");
    Landmarks node;
    node.main_loop();
    return 0;
}