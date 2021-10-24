/// \file fake_turtle.cpp
/// \brief contains a node called fake_turtle to simulate a differential drive robot
/// using the DiffDrive class
///
/// PARAMETERS:
///     left_wheel_joint : string used for publishing joint_state_message
///     right_wheel_joint : string used for publishing joint_state_message
///     wheel_rad : the radius of the robot's wheels
///     wheel_base : the distance between the robot's wheels
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

class TubeWorld {
    private:

        int frequency = 50;
        bool latch = true;

        // variables from parameter server
        double max_range;
        double wheel_base, wheel_rad;
        double tube_rad, tube_var;

        double twist_noise;
        double slip_min, slip_max;
        double robot_rad;

        double max_scan_range, min_scan_range;
        double angle_incr, sample_num;
        double scan_res, scan_noise;
        double wall_width, wall_height;

        std::string odom_frame_id, world_frame_id, turtle_frame_id, scanner_frame_id;
        std::string left_wheel_joint, right_wheel_joint;
        std::vector<double> t1_loc, t2_loc, t3_loc, t4_loc, t5_loc, t6_loc;
        std::vector<std::vector<double>> tube_locs;

        // ros objects (node handle, subs, pubs, services, etc)
        ros::NodeHandle n;

        ros::Publisher marker_true_pub;
        ros::Publisher marker_rel_pub;
        ros::Publisher wall_pub;
        ros::Publisher joint_pub;
        ros::Publisher path_pub;
        ros::Publisher lidar_pub;

        ros::Subscriber twist_sub;
        tf2_ros::TransformBroadcaster broadcaster;

        // variables
        geometry_msgs::Twist twist_msg;
        rigid2d::Twist2D desired_twist;
        bool twist_received = false;

        rigid2d::DiffDrive ninja_turtle;

        sensor_msgs::JointState joint_msg;
        nav_msgs::Path path;
        sensor_msgs::LaserScan scan_msg;

    public:

        TubeWorld() {
            load_parameters();

            marker_true_pub = n.advertise<visualization_msgs::MarkerArray>("/ground_truth", 10, latch);
            marker_rel_pub = n.advertise<visualization_msgs::MarkerArray>("/fake_sensor", 10);
            wall_pub = n.advertise<visualization_msgs::Marker>("/wall", 10, latch);
            joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
            path_pub = n.advertise<nav_msgs::Path>("/real_path", 10);
            lidar_pub = n.advertise<sensor_msgs::LaserScan>("/scan", 10);
            twist_sub = n.subscribe("/cmd_vel", 100, &TubeWorld::twist_callback, this);
        }

        void load_parameters() {
            /***********
             * Read parameters from parameter server
             * ********/
            n.getParam("max_range", max_range);
            n.getParam("wheel_base", wheel_base);
            n.getParam("wheel_radius", wheel_rad);
            n.getParam("odom_frame_id", odom_frame_id);
            n.getParam("scanner_frame_id", scanner_frame_id);
            n.getParam("left_wheel_joint", left_wheel_joint);
            n.getParam("right_wheel_joint", right_wheel_joint);

            n.getParam("tube1_location", t1_loc);
            n.getParam("tube2_location", t2_loc);
            n.getParam("tube3_location", t3_loc);
            n.getParam("tube4_location", t4_loc);
            n.getParam("tube5_location", t5_loc);
            n.getParam("tube6_location", t6_loc);

            n.getParam("tube_radius", tube_rad);
            n.getParam("tube_var", tube_var);
            n.getParam("world_frame_id", world_frame_id);
            n.getParam("turtle_frame_id", turtle_frame_id);
            n.getParam("twist_noise", twist_noise);
            n.getParam("slip_min", slip_min);
            n.getParam("slip_max", slip_max);
            n.getParam("robot_radius", robot_rad);

            n.getParam("maximum_range", max_scan_range);
            n.getParam("minimum_range", min_scan_range);
            n.getParam("angle_increment", angle_incr);
            n.getParam("sample_num", sample_num);
            n.getParam("resolution", scan_res);
            n.getParam("noise_level", scan_noise);
            n.getParam("wall_width", wall_width);
            n.getParam("wall_height", wall_height);

            tube_locs.push_back(t1_loc);
            tube_locs.push_back(t2_loc);
            tube_locs.push_back(t3_loc);
            tube_locs.push_back(t4_loc);
            tube_locs.push_back(t5_loc);
            tube_locs.push_back(t6_loc);

            return;
        }

        void twist_callback(const geometry_msgs::Twist& msg) {
            // std::cout << "twist received!" << std::endl;
            std::normal_distribution<> gaus_twist(0, twist_noise);

            // create desired twist based on the message, add gaussian noise
            desired_twist.dth = msg.angular.z + gaus_twist(get_random());
            desired_twist.dx = msg.linear.x + gaus_twist(get_random());
            desired_twist.dy = msg.linear.y;

            twist_received = true;

            return;
        }

        void set_markers(void) {
            std::cout << "setting ground truth markers!!" << std::endl;
            visualization_msgs::MarkerArray marker_array;

            tf2::Quaternion marker_quat;
            marker_quat.setRPY(0.0, 0.0, 0.0);
            geometry_msgs::Quaternion markerQuat = tf2::toMsg(marker_quat);

            ros::Time current_time = ros::Time::now();

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
                marker.scale.x = tube_rad*2;
                marker.scale.y = tube_rad*2;
                marker.scale.z = 0.2;
                marker.color.a = 1.0;
                marker.color.r = 195. / 255.;
                marker.color.g = 205. / 255.;
                marker.color.b = 230. / 255.;
                marker.frame_locked = true;
                marker_array.markers.push_back(marker);
            }

            marker_true_pub.publish(marker_array);

            visualization_msgs::Marker wall;
            wall.id = 0;
            geometry_msgs::Point up_left, up_right, low_left, low_right;

            up_left.x = -wall_width/2;
            up_left.y = wall_height/2;
            
            up_right.x = wall_width/2;
            up_right.y = wall_height/2;

            low_left.x = -wall_width/2;
            low_left.y = -wall_height/2;

            low_right.x = wall_width/2;
            low_right.y = -wall_height/2;

            std::vector<geometry_msgs::Point> wall_points{up_left, up_right, up_right, low_right, low_left, up_left};

            wall.header.frame_id = world_frame_id;
            wall.header.stamp = current_time;
            wall.ns = "real";
            wall.type = 4;

            wall.action = visualization_msgs::Marker::ADD;
            
            for (auto w : wall_points)
            {
                wall.points.push_back(w);
            }
            wall.scale.x = 0.01;
            wall.color.a = 1;
            wall.color.r = 250. / 255.;
            wall.color.g = 192. / 255.;
            wall.color.b = 221. / 255.;
            wall.frame_locked = true;

            marker_array.markers.push_back(wall);

            wall_pub.publish(wall);
            return;
        }

        void set_rel_markers(void) {
            using namespace rigid2d;

            // find the transformation between the world frame and the turtle frame
            Vector2D trans_robot(ninja_turtle.getX(), ninja_turtle.getY());
            Transform2D T_wt = Transform2D(trans_robot, ninja_turtle.getTh());
            Transform2D T_tw = T_wt.inv();

            ros::Time current_time = ros::Time::now();

            visualization_msgs::MarkerArray marker_array_relative;

            tf2::Quaternion marker_quat;
            marker_quat.setRPY(0.0, 0.0, 0.0);
            geometry_msgs::Quaternion markerQuat = tf2::toMsg(marker_quat);

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
                double rel_distance = sqrt(pow(loc[0] - ninja_turtle.getX(), 2) + pow(loc[1] - ninja_turtle.getY(), 2));
                if (rel_distance > max_range) {
                    marker_relative.action = visualization_msgs::Marker::DELETE;
                }
                // otherwise add it to the marker array
                else {
                    marker_relative.action = visualization_msgs::Marker::ADD;
                }

                // find coordinates of the tube relative to the turtle
                Vector2D tube_w(loc[0], loc[1]);
                Vector2D tube_t = T_tw(tube_w);

                marker_relative.pose.position.x = tube_t.x + tube_var;
                marker_relative.pose.position.y = tube_t.y + tube_var;
                marker_relative.pose.position.z = 0.1;
                marker_relative.pose.orientation = markerQuat;
                marker_relative.scale.x = tube_rad*2;
                marker_relative.scale.y = tube_rad*2;
                marker_relative.scale.z = 0.2;
                marker_relative.color.a = 1.0;
                marker_relative.color.r = 244. / 255.;
                marker_relative.color.g = 238. / 255.;
                marker_relative.color.b = 177. / 255.;
                marker_relative.frame_locked = true;

                marker_array_relative.markers.push_back(marker_relative);
            }

            marker_rel_pub.publish(marker_array_relative);
            return;
        }

        void broadcast_world_to_turtle_tf(void) {
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, ninja_turtle.getTh());

            geometry_msgs::Quaternion quat = tf2::toMsg(quaternion);

            geometry_msgs::TransformStamped trans;
            trans.header.stamp = ros::Time::now();
            trans.header.frame_id = world_frame_id;
            trans.child_frame_id = turtle_frame_id;

            trans.transform.translation.x = ninja_turtle.getX();
            trans.transform.translation.y = ninja_turtle.getY();
            trans.transform.translation.z = 0.0;
            trans.transform.rotation = quat;

            broadcaster.sendTransform(trans);
            return;
        }

        void check_collision(void) {
            // check the distance betwen the center of the robot and the center of each tube
            for (auto loc : tube_locs) {
                double dx = loc[0] - ninja_turtle.getX();
                double dy = loc[1] - ninja_turtle.getY();

                double dist = sqrt(pow(dx,2) + pow(dy,2));

                if (dist <= (tube_rad + robot_rad)) {
                    std::cout << "robot has collided with tube!!" << std::endl;
                    double move_x = dy / dist;
                    double move_y = -dx / dist;

                    // have the robot move (slip) along the angent line
                    ninja_turtle.changeConfig(move_x / 50, move_y / 50);
                }
            }
            return;
        }

        void draw_trajectory(void) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = ninja_turtle.getX();
            pose_stamped.pose.position.y = ninja_turtle.getY();
            pose_stamped.pose.orientation.z = ninja_turtle.getTh();

            path.header.stamp = ros::Time::now();
            path.header.frame_id = world_frame_id;
            path.poses.push_back(pose_stamped);
            path_pub.publish(path);
            return;
        }

        // still need to fix lidar function
        void simulate_lidar_scanner(void) {
            using namespace rigid2d;

            scan_msg.header.frame_id = turtle_frame_id;
            scan_msg.header.stamp = ros::Time::now();
            scan_msg.angle_min = 0.0;
            scan_msg.angle_max = 2*PI;
            scan_msg.angle_increment = 2*PI / sample_num;
            scan_msg.range_min = min_scan_range;
            scan_msg.range_max = max_scan_range;
            
            std::vector<float> lidar_ranges(sample_num, max_scan_range+1);

            for (auto tube : tube_locs) {
                double xt = tube[0];
                double yt = tube[1];

                // get the coordinates of the robot relative to the tube
                double x1 = ninja_turtle.getX() - xt;
                double y1 = ninja_turtle.getY() - yt;

                int tube_angle = round(rad2deg(atan2(yt-y1, xt-x1)));

                for (int i = tube_angle - 27; i < tube_angle + 27; i++) {
                    double x2 = x1 + max_scan_range * cos(deg2rad(i));
                    double y2 = y1 + max_scan_range * sin(deg2rad(i));

                    double dx = x2 - x1;
                    double dy = y2 - y1;
                    double dr = sqrt(pow(dx,2) + pow(dy,2));
                    double det = x1*y2 - x2*y1;
                    double dis = (pow(tube_rad,2) * pow(dr,2)) - pow(det,2);
                    double distance;

                    if (fabs(dis) < 1e-5) {
                        double inter_x = (det * dy) / pow(dr,2);
                        double inter_y = -(det * dx) / pow(dr,2);
                        distance = sqrt(pow(inter_x-x1,2) + pow(inter_y-y1,2));
                    }
                    else if (dis > 0) {
                        double inter_x1 = ((det*dy) + ((dy/fabs(dy))*dx*sqrt((pow(tube_rad,2)*pow(dr,2)) - pow(det,2))))/pow(dr,2);
                        double inter_y1 = (-(det*dx) + fabs(dy)*sqrt((pow(tube_rad,2)*pow(dr,2)) - pow(det,2))) / pow(dr,2);
                        double dist1 = sqrt(pow(inter_x1-x1,2) + pow(inter_y1-y1,2));

                        double inter_x2 = ((det*dy) - ((dy/fabs(dy))*dx*sqrt((pow(tube_rad,2)*pow(dr,2)) - pow(det,2))))/pow(dr,2);
                        double inter_y2 = (-(det*dx) - fabs(dy)*sqrt((pow(tube_rad,2)*pow(dr,2)) - pow(det,2))) / pow(dr,2);
                        double dist2 = sqrt(pow(inter_x2-x1,2) + pow(inter_y2-y1,2));

                        distance = std::min(dist1, dist2);
                    }
                    else {
                        distance = max_scan_range + 1;
                    }

                    int ind = (i - int(rad2deg(ninja_turtle.getTh()))) % 360;
                    if (ind < 0) ind += 360;

                    if (distance < lidar_ranges[ind]) {
                        lidar_ranges[ind] = distance;
                    }
                }
            }
            scan_msg.ranges = lidar_ranges;
            scan_msg.intensities = std::vector<float> (360, 4000);

            lidar_pub.publish(scan_msg);
        }
        
        void main_loop() {
            using namespace rigid2d;

            ros::Rate loop_rate(frequency);
            ros::Time current_time = ros::Time::now();
            ros::Time last_time = ros::Time::now();

            double slip_mean = (slip_min + slip_max) / 2;
            double slip_var = slip_max - slip_mean;

            std::normal_distribution<> slip_noise(slip_mean, slip_var);

            std::normal_distribution<> gaus_twist(0, twist_noise);

            ninja_turtle = rigid2d::DiffDrive(wheel_base, wheel_rad, 0.0,
                                                                     0.0, 
                                                                     0.0, 
                                                                     0.0, 
                                                                     0.0);

            joint_msg.header.stamp = current_time;
            joint_msg.header.frame_id = turtle_frame_id;
            joint_msg.name.push_back(left_wheel_joint);
            joint_msg.name.push_back(right_wheel_joint);

            joint_msg.position.push_back(0.0);
            joint_msg.position.push_back(0.0);

            joint_pub.publish(joint_msg);

            set_markers();

            while (ros::ok()) {

                current_time = ros::Time::now();

                broadcast_world_to_turtle_tf();
                
                if (twist_received) {
                    check_collision();
                    
                    // find the wheel velocities required to achieve that twist
                    wheelVel wheel_vel = ninja_turtle.convertTwist(desired_twist);

                    // populate the sensor messages, add wheel slip noise
                    joint_msg.header.stamp = current_time;
                    joint_msg.header.frame_id = turtle_frame_id;

                    joint_msg.position[0] += wheel_vel.uL * (current_time - last_time).toSec();
                    joint_msg.position[1] += wheel_vel.uR * (current_time - last_time).toSec();

                    joint_pub.publish(joint_msg);

                    // update the configuration of the diff-drive robot based on new wheel angles
                    ninja_turtle(joint_msg.position[0] + wheel_vel.uL * slip_noise(get_random()), 
                                 joint_msg.position[1] + wheel_vel.uR * slip_noise(get_random()));

                    broadcast_world_to_turtle_tf();
                    
                    set_rel_markers();

                    draw_trajectory();

                    simulate_lidar_scanner();

                    twist_received = false;
                }

                last_time = current_time;
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tube_world");
    TubeWorld node;
    node.main_loop();
    return 0;
}