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
        ros::Publisher odom_pub;
        ros::Publisher slam_path_pub;
        ros::Publisher odom_path_pub;
        ros::Publisher slam_landmarks_pub;
        ros::Subscriber landmark_sub;
        ros::Subscriber joint_sub;
        tf2_ros::TransformBroadcaster broadcaster;

        // parameters
        std::string left_wheel_joint, right_wheel_joint;
        std::string world_frame_id, turtle_frame_id, map_frame_id, odom_frame_id, body_frame_id;

        double wheel_base, wheel_rad;
        double tube_rad;
        double min_range, max_range;

        // variables
        int frequency = 5;
        bool landmarks_received = false;
        bool joint_states_received = false;
        int seen_landmarks;
        int total_landmarks = 6;

        std::vector<double> r_vec, q_vec;
        sensor_msgs::JointState joint_state_msg;
        visualization_msgs::MarkerArray landmarks;
        nav_msgs::Path odom_path;
        rigid2d::Twist2D twist;

        rigid2d::DiffDrive odom_model;
        
        slam_library::ExtendedKalman extended_kalman_filter;
        arma::colvec state_estimation;

    public:
        EKFSlam() {
            load_parameters();

            odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
            slam_path_pub = n.advertise<nav_msgs::Path>("/slam_path", 10);
            odom_path_pub = n.advertise<nav_msgs::Path>("/odom_path", 10);
            slam_landmarks_pub = n.advertise<visualization_msgs::MarkerArray>("/estimated_landmarks", 10);
            landmark_sub = n.subscribe("/real_sensor", 10, &EKFSlam::landmark_callback, this);
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
   
            joint_state_msg = msg;

            return;
        }

        void initialize_slam(void) {
            std::cout << "slam initialized" << std::endl;
            using namespace arma;

            // robot state
            colvec robot_state(3);
            robot_state(0) = odom_model.getTh();
            robot_state(1) = odom_model.getX();
            robot_state(2) = odom_model.getY();

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
            seen_landmarks = extended_kalman_filter.getSeenLandmarks();
        }

        void draw_odom_path(void) {
            // add pose to the path
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = odom_model.getX();
            pose_stamped.pose.position.y = odom_model.getY();
            pose_stamped.pose.orientation.z = odom_model.getTh();

            odom_path.header.stamp = ros::Time::now();
            odom_path.header.frame_id = world_frame_id;
            odom_path.poses.push_back(pose_stamped);
            odom_path_pub.publish(odom_path);
            return;
        }

        void broadcast_map2odom_tf(void) {
            using namespace rigid2d;

            // transformation from odom to body frame id
            Vector2D v;
            v.x = odom_model.getX();
            v.y = odom_model.getY();
            Transform2D T_ob(v, odom_model.getTh());

            arma::colvec state_estimate = extended_kalman_filter.getStateVector();
            // transformation from map to body frame id
            v.x = state_estimate(1);
            v.y = state_estimate(2);
            Transform2D T_mb(v, state_estimate(0));

            // transformation from map to odom frame id
            Transform2D T_mo = T_mb * T_ob.inv();

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, normalize_angle(asin(T_mo.getSinTh())));

            geometry_msgs::Quaternion quat = tf2::toMsg(q);

            geometry_msgs::TransformStamped trans;
            trans.header.stamp = ros::Time::now();
            trans.header.frame_id = map_frame_id;
            trans.child_frame_id = odom_frame_id;

            trans.transform.translation.x = T_mo.getX();
            trans.transform.translation.y = T_mo.getY();
            trans.transform.translation.z = 0.0;
            trans.transform.rotation = quat;

            broadcaster.sendTransform(trans);
            return;
        }

        void broadcast_odom2body_tf(void) {
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, odom_model.getTh());
            geometry_msgs::Quaternion quat = tf2::toMsg(q);

            geometry_msgs::TransformStamped trans;
            trans.header.stamp = ros::Time::now();
            trans.header.frame_id = odom_frame_id;
            trans.child_frame_id = body_frame_id;

            trans.transform.translation.x = odom_model.getX();
            trans.transform.translation.y = odom_model.getY();
            trans.transform.translation.z = 0.0;
            trans.transform.rotation = quat;

            broadcaster.sendTransform(trans);
            return;
        }

        void main_loop(void) {
            using namespace rigid2d;
            using namespace arma;
            using namespace slam_library;

            ros::Rate loop_rate(frequency);

            initialize_slam();

            odom_model = rigid2d::DiffDrive(wheel_base, wheel_rad, 0.0,
                                                                   0.0,
                                                                   0.0,
                                                                   0.0,
                                                                   0.0);

            while (ros::ok()) {
                broadcast_odom2body_tf();
                broadcast_map2odom_tf();

                state_estimation = extended_kalman_filter.getStateVector();
                seen_landmarks = extended_kalman_filter.getSeenLandmarks();
                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\r" << std::endl;
                std::cout << "There have been " << seen_landmarks << " seen landmarks\r" << std::endl;
                std::cout << "state estimation\r" << std::endl;
                for (auto s : state_estimation) {
                    std::cout << s << "\r" << std::endl;
                }

                if (joint_states_received) {
                    // get twist command
                    twist = odom_model.getTwist(joint_state_msg.position[0], joint_state_msg.position[1]);
                    odom_model(joint_state_msg.position[0], joint_state_msg.position[1]);
                    draw_odom_path();

                    // prediction step
                    extended_kalman_filter.predict(twist);

                    if (landmarks_received) {
                        
                        // loop through each landmark
                        for (auto landmark : landmarks.markers) {

                            // get cartesian distances between the landmark and turtle
                            double x = landmark.pose.position.x;
                            double y = landmark.pose.position.y;

                            // calculate z_i
                            colvec z_i = cartesian2polar(x, y);

                            std::cout << "z_i dist: " << z_i(0) << " angle: " << z_i(1) << "\r" << std::endl;

                            // perform data association
                            int id = extended_kalman_filter.associateLandmark(z_i);

                            std::cout << "measurement associated with landmark id " << id << "\r" << std::endl;

                            if (id > seen_landmarks) {
                                extended_kalman_filter.initializeLandmark(z_i, id);
                            }
                            else if (id < 0) {
                                continue;
                            }
                            else if (id > total_landmarks) {
                                std::cout << "max landmarks initialized\r" << std::endl;
                                break;
                                continue;
                            }

                            extended_kalman_filter.update(twist, z_i, id);
                        }

                        state_estimation = extended_kalman_filter.getStateVector();

                        visualization_msgs::MarkerArray estimated_landmarks;

                        for (int i = 1; i <= seen_landmarks; i++) {
                            visualization_msgs::Marker m;
                            m.header.frame_id = map_frame_id;
                            m.header.stamp = ros::Time::now();
                            m.ns = "estimate";
                            m.id = i;
                            m.type = visualization_msgs::Marker::CYLINDER;
                            m.action = visualization_msgs::Marker::ADD;
                            m.pose.position.x = state_estimation(3+2*(i-1));
                            m.pose.position.y = state_estimation(4+2*(i-1));
                            m.pose.position.z = 0.125;

                            tf2::Quaternion m_quat;
                            m_quat.setRPY(0.0, 0.0, 0.0);
                            geometry_msgs::Quaternion quat = tf2::toMsg(m_quat);

                            m.pose.orientation = quat;
                            m.scale.x = 2*tube_rad;
                            m.scale.y = 2*tube_rad;
                            m.scale.z = 0.25;
                            m.color.a = 1.0;
                            m.color.r = 38./255.;
                            m.color.g = 91./255.;
                            m.color.b = 95./255.;
                            m.frame_locked = true;

                            estimated_landmarks.markers.push_back(m);
                        }

                        slam_landmarks_pub.publish(estimated_landmarks);

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