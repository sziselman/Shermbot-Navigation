/// \file slam.cpp
/// \brief contains a node called slam that will implement EKF SLAM
///
/// PARAMETERS:
///     tube_radius: the radius of the tube / landmarks
///     robot_radius: the radius of the turtlebot
///     R : 2x2 sensor noise matrix
///     Q : 3x3 process noise matrix
///     tube_locations : the (x,y) locations of each tube / landmark
/// PUBLISHES:  /slam_path (nav_msgs::Path)
///             /odom_path (nav_msgs::Path)
/// SUBSCRIBES: /joint_states (sensor_msgs::JointState)
///             /fake_sensor (visualization_msgs::MarkerArray)
/// SERVICES: set_pose : Sets the pose of the turtlebot's configuration

#include <ros/ros.h>

#include <rigid2d/set_pose.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

#include "nuslam/slam_library.hpp"

#include <armadillo>
#include <string>
#include <iostream>

class EKFSlam {
    private:
        // ros variables (pubs, subs, etc.)
        ros::NodeHandle n;
        ros::Publisher slam_path_pub;
        ros::Publisher odom_path_pub;
        ros::Subscriber landmark_sub;
        ros::Subscriber joint_sub;

        // parameters
        std::string left_wheel_joint, right_wheel_joint;
        std::string world_frame_id, turtle_frame_id, map_frame_id, odom_frame_id, body_frame_id;

        double wheel_base, wheel_rad;
        double tube_rad;
        double min_range, max_range;

        // variables
        int frequency = 10;
        bool landmarks_received = false;
        bool joint_states_received = false;
        int seen_landmarks = 0;
        int total_landmarks = 6;

        std::vector<double> r_vec, q_vec;
        sensor_msgs::JointState joint_state_msg;
        visualization_msgs::MarkerArray landmarks;

        rigid2d::DiffDrive actual_robot = rigid2d::DiffDrive(wheel_base, wheel_rad, 0.0,
                                                                                    0.0,
                                                                                    0.0,
                                                                                    0.0,
                                                                                    0.0);

        rigid2d::DiffDrive slam_estimate = rigid2d::DiffDrive(wheel_base, wheel_rad, 0.0,
                                                                                    0.0,
                                                                                    0.0,
                                                                                    0.0,
                                                                                    0.0);
        
        slam_library::ExtendedKalman extended_kalman_filter;

    public:
        EKFSlam() {
            load_parameters();

            slam_path_pub = n.advertise<nav_msgs::Path>("/slam_path", 10);
            odom_path_pub = n.advertise<nav_msgs::Path>("/odom_path", 10);
            landmark_sub = n.subscribe("/scan", 10, &EKFSlam::landmark_callback, this);
            joint_sub = n.subscribe("/joint_states", 10, &EKFSlam::joint_state_callback, this);
        }

        void load_parameters(void) {
            n.getParam("wheel_base", wheel_base);
            n.getParam("wheel_radius", wheel_rad);

            n.getParam("map_frame_id", map_frame_id);
            n.getParam("odom_frame_id", odom_frame_id);
            n.getParam("body_frame_id", body_frame_id);
            n.getParam("left_wheel_joint", left_wheel_joint);
            n.getParam("right_wheel_joint", right_wheel_joint);
            n.getParam("world_frame_id", world_frame_id);

            n.getParam("tube_radius", tube_rad);

            n.getParam("R", r_vec);
            n.getParam("Q", q_vec);
            return;
        }

        void landmark_callback(const visualization_msgs::MarkerArray &msg) {
            landmarks_received = true;
            landmarks = msg;
            // implement data association (mahalanobis distance)
            return;
        }

        void joint_state_callback(const sensor_msgs::JointState &msg) {
            joint_states_received = true;
            // update the configuration of the actual robot based on the joint state messages
            actual_robot(msg.position[0], msg.position[1]);

            joint_state_msg = msg;

            return;
        }

        void initialize_slam(void) {
            using namespace arma;

            // robot state
            colvec robot_state(3);
            robot_state(0) = actual_robot.getTh();
            robot_state(1) = actual_robot.getX();
            robot_state(2) = actual_robot.getY();

            // map state, everything initialized to 0 since we don't know any landmarks
            colvec map_state(2*total_landmarks);
            for (int i = 0; i < 2*total_landmarks; i++) {
                map_state(i) = 0;
            }

            // Q matrix
            mat Q(3,3);
            for (auto q : q_vec) {
                Q(q) = q_vec[q];
            }

            // R matrix
            mat R(2,2);
            for (auto r : r_vec) {
                R(r) = r_vec[r];
            }

            extended_kalman_filter = slam_library::ExtendedKalman(robot_state, map_state, Q, R);
        }

        void main_loop(void) {
            using namespace rigid2d;
            using namespace arma;
            using namespace slam_library;

            ros::Rate loop_rate(frequency);

            initialize_slam();

            while (ros::ok()) {

                // if joint states have been received, the configuration of the actual turtle gets updated
                if (joint_states_received) {

                    // if landmarks have been received, implement data association
                    if (landmarks_received) {
                        
                        // get the twist that corresponds to the updated joint states
                        Twist2D twist = slam_estimate.getTwist(joint_state_msg.position[0], joint_state_msg.position[1]);

                        extended_kalman_filter.predict(twist);

                        // for each measurement i
                        for (auto landmark : landmarks.markers) {
                            // calculate z_i from landmark
                            colvec measurement(2);
                            double x = landmark.pose.position.x;
                            double y = landmark.pose.position.y;

                            colvec z_i = cartesian2polar(x, y);

                            // get the associated landmark id
                            int landmark_id = extended_kalman_filter.associateLandmark(z_i);

                            // if the landmark is out of range, ignore it
                            if (landmark_id < 0) {
                                continue;
                            }
                            // if a landmark is greater than the number of seen landmarks (new landmark) initialize it
                            else if (landmark_id > seen_landmarks) {
                                extended_kalman_filter.initializeLandmark(z_i, landmark_id);
                                seen_landmarks++;
                            }


                        }
                        landmarks_received = false;
                    }

                    joint_states_received = false;
                }
                

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "slam");
    EKFSlam node;
    node.main_loop();
    return 0;
}