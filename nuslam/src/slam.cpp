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

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

#include <nuslam/slam_library.hpp>

#include <armadillo>
#include <string>
#include <iostream>

/**********
 * Declare global variables
 * *******/
static rigid2d::DiffDrive ninjaTurtle;
static rigid2d::DiffDrive teenageMutant;

static sensor_msgs::JointState joint_state_msg;
static visualization_msgs::MarkerArray marker_array, marker_array_fake;

static double wheelBase, wheelRad;
static bool jointState_flag = false;
static bool markerArray_flag = false;
static bool markerArrayFake_flag = false;

/**********
 * Helper Functions
 * *******/
void jointStateCallback(const sensor_msgs::JointState msg);
void sensorCallback(const visualization_msgs::MarkerArray array);
void fakeSensorCallback(const visualization_msgs::MarkerArray array);
bool setPose(rigid2d::set_pose::Request & req, rigid2d::set_pose::Response & res);

/*********
 * Main Function
 * ******/
int main(int argc, char* argv[])
{
    using namespace rigid2d;
    using namespace slam_library;
    using namespace arma;

    /*********
     * Initialize the node & node handle
     * ******/
    ros::init(argc, argv, "slam");
    ros::NodeHandle n;

    /*********
     * Declare local variables
     * ******/
    std::string left_wheel_joint, right_wheel_joint, world_frame_id, turtle_frame_id;
    std::string map_frame_id, odom_frame_id, body_frame_id;
    std::vector<double> tube1_loc, tube2_loc, tube3_loc, tube4_loc, tube5_loc, tube6_loc;
    std::vector<double> rVec, qVec;
    double tubeRad;

    int frequency=10;
    int num = 6;

    nav_msgs::Path odom_path;
    nav_msgs::Path slam_path;
    tf2_ros::TransformBroadcaster broadcaster;

    /*********
     * Read parameters from parameter server
     * ******/
    n.getParam("wheel_base", wheelBase);
    n.getParam("wheel_radius", wheelRad);
    n.getParam("map_frame_id", map_frame_id);
    n.getParam("odom_frame_id", odom_frame_id);
    n.getParam("body_frame_id", body_frame_id);
    n.getParam("left_wheel_joint", left_wheel_joint);
    n.getParam("right_wheel_joint", right_wheel_joint);
    n.getParam("world_frame_id", world_frame_id);
    n.getParam("tube1_location", tube1_loc);
    n.getParam("tube2_location", tube2_loc);
    n.getParam("tube3_location", tube3_loc);
    n.getParam("tube4_location", tube4_loc);
    n.getParam("tube5_location", tube5_loc);
    n.getParam("tube6_location", tube6_loc);
    n.getParam("tube_radius", tubeRad);
    n.getParam("R", rVec);
    n.getParam("Q", qVec);

    /*********
     * Define publishers, subscribers, services and clients
     ********/
    ros::Publisher slamPath_pub = n.advertise<nav_msgs::Path>("/slam_path", frequency);
    ros::Publisher odomPath_pub = n.advertise<nav_msgs::Path>("/odom_path", frequency);

    ros::Subscriber joint_sub = n.subscribe("/joint_states", frequency, jointStateCallback);
    ros::Subscriber sensor_sub = n.subscribe("/real_sensor", frequency, sensorCallback);
    ros::Subscriber fake_sensor_sub = n.subscribe("/fake_sensor", frequency, fakeSensorCallback);

    ros::ServiceServer setPose_service = n.advertiseService("set_pose", setPose);
    ros::ServiceClient setPose_client = n.serviceClient<rigid2d::set_pose>("set_pose");

    ros::Rate loop_rate(frequency);
    /*********
     * Set initial parameters of the differential drive robot to 0
     * ******/
    ninjaTurtle = DiffDrive(wheelBase, wheelRad, 0.0, 0.0, 0.0, 0.0, 0.0);
    teenageMutant = DiffDrive(wheelBase, wheelRad, 0.0, 0.0, 0.0, 0.0, 0.0);

    /*********
     * Create EKF SLAM object
     * ******/
    colvec mapState(2*num);
    mapState(0) = tube1_loc[0];
    mapState(1) = tube1_loc[1];
    mapState(2) = tube2_loc[0];
    mapState(3) = tube2_loc[1];
    mapState(4) = tube3_loc[0];
    mapState(5) = tube3_loc[1];
    mapState(6) = tube4_loc[0];
    mapState(7) = tube4_loc[1];
    mapState(8) = tube5_loc[0];
    mapState(9) = tube5_loc[1];
    mapState(10) = tube6_loc[0];
    mapState(11) = tube6_loc[1];

    colvec robotState(3);
    robotState(0) = ninjaTurtle.getTh();
    robotState(1) = ninjaTurtle.getX();
    robotState(2) = ninjaTurtle.getY();

    mat Q(3, 3);
    for (int i = 0; i < int(qVec.size()); ++i)
    {
        Q(i) = qVec[i];
    }

    mat R(2,2);
    for (int j = 0; j < int(rVec.size()); ++j)
    {
        R(j) = rVec[j];
    }

    ExtendedKalman raphael = ExtendedKalman(robotState, mapState, Q, R);

    while (ros::ok())
    {
        ros::spinOnce();

        ros::Time current_time = ros::Time::now();

        /**********
         * If a joint state message is received
         * *******/
        if (jointState_flag)
        {
            /***********
             * Get twist from new wheel angles and update configuration
             * ********/
            ninjaTurtle(joint_state_msg.position[0], joint_state_msg.position[1]);

            /**********
             * If a marker array from real sensor is received
             * *******/
            if (markerArray_flag)
            {
                // // get velocities from new wheel angle
                // made a separate diffdrive object since marker array messages may be sent at a different freuqancy
                Twist2D slam_twist = teenageMutant.getTwist(joint_state_msg.position[0], joint_state_msg.position[1]);

                teenageMutant(joint_state_msg.position[0], joint_state_msg.position[1]);

                /***********
                 *  predict: update the estimate using the model
                 * ********/
                // update the estimate using the model
                colvec prevState(3+2*num);
                prevState = raphael.getStateVec();

                colvec newState(3+2*num);
                newState = raphael.g(prevState, slam_twist);

                // propagate the uncertainty using the linearized state transition model
                mat Q_bar(3+2*num, 3+2*num);
                Q_bar = raphael.Q_bar();

                mat A(3+2*num, 3+2*num);
                A = raphael.getA(prevState, slam_twist);

                mat cov(3+2*num, 3+2*num);
                cov = raphael.getCov();

                mat covNew(3+2*num, 3+2*num);
                covNew = A * cov * A.t() + Q_bar;

                raphael.updateStateVec(newState);
                
                raphael.updateCov(covNew);
                
                // for loop that goes through each marker that was measured
                for (auto marker: marker_array.markers)
                {
                    // int j = marker.id + 1;

                    // put the marker (x,y) location in range-bearing form
                    colvec rangeBearing(2);
                    rangeBearing = RangeBearing(marker.pose.position.x, marker.pose.position.y);

                    // data association
                    int j = raphael.DataAssociation(rangeBearing);

                    // compute theoretical measurements, given the current state estimate
                    colvec z_hat(2);
                    z_hat = raphael.h(j);

                    // compute the Kalman gain from the linearized measurement model
                    mat K_j(3+2*num, 2);
                    K_j = raphael.KalmanGain(j);

                    // compute the posterior state update
                    colvec currentEstimate(3+2*num);
                    currentEstimate = raphael.getStateVec();

                    colvec stateUpdate(3+2*num);
                    colvec z_diff(2);
                    z_diff = rangeBearing - z_hat;
                    z_diff(1) = normalize_angle(z_diff(1));
                    stateUpdate = currentEstimate + K_j * z_diff;

                    // compute the posterior covariance
                    mat currentCov(3+2*num, 3+2*num);
                    currentCov = raphael.getCov();

                    mat Identity(3+2*num, 3+2*num, fill::eye);

                    mat H_j(2, 3+2*num);
                    H_j = raphael.getH(j);

                    mat newCov(3+2*num, 3+2*num);
                    newCov = (Identity - K_j * H_j) * currentCov;

                    // update the state and covariance
                    raphael.updateStateVec(stateUpdate);
                    raphael.updateCov(newCov);
                }
                
                markerArray_flag = false;
            }

            /**********
             * If a marker array from fake sensor is received
             * *******/
            if (markerArrayFake_flag)
            {
                // // get velocities from new wheel angle
                // made a separate diffdrive object since marker array messages may be sent at a different freuqancy
                Twist2D slam_twist = teenageMutant.getTwist(joint_state_msg.position[0], joint_state_msg.position[1]);

                teenageMutant(joint_state_msg.position[0], joint_state_msg.position[1]);

                /***********
                 *  predict: update the estimate using the model
                 * ********/
                // update the estimate using the model
                colvec prevState(3+2*num);
                prevState = raphael.getStateVec();

                colvec newState(3+2*num);
                newState = raphael.g(prevState, slam_twist);

                // propagate the uncertainty using the linearized state transition model
                mat Q_bar(3+2*num, 3+2*num);
                Q_bar = raphael.Q_bar();

                mat A(3+2*num, 3+2*num);
                A = raphael.getA(prevState, slam_twist);

                mat cov(3+2*num, 3+2*num);
                cov = raphael.getCov();

                mat covNew(3+2*num, 3+2*num);
                covNew = A * cov * A.t() + Q_bar;

                raphael.updateStateVec(newState);
                
                raphael.updateCov(covNew);
                
                // for loop that goes through each marker that was measured
                for (auto marker: marker_array.markers)
                {
                    int j = marker.id + 1;

                    // put the marker (x,y) location in range-bearing form
                    colvec rangeBearing(2);
                    rangeBearing = RangeBearing(marker.pose.position.x, marker.pose.position.y);

                    // compute theoretical measurements, given the current state estimate
                    colvec z_hat(2);
                    z_hat = raphael.h(j);

                    // compute the Kalman gain from the linearized measurement model
                    mat K_j(3+2*num, 2);
                    K_j = raphael.KalmanGain(j);

                    // compute the posterior state update
                    colvec currentEstimate(3+2*num);
                    currentEstimate = raphael.getStateVec();

                    colvec stateUpdate(3+2*num);
                    colvec z_diff(2);
                    z_diff = rangeBearing - z_hat;
                    z_diff(1) = normalize_angle(z_diff(1));
                    stateUpdate = currentEstimate + K_j * z_diff;

                    // compute the posterior covariance
                    mat currentCov(3+2*num, 3+2*num);
                    currentCov = raphael.getCov();

                    mat Identity(3+2*num, 3+2*num, fill::eye);

                    mat H_j(2, 3+2*num);
                    H_j = raphael.getH(j);

                    mat newCov(3+2*num, 3+2*num);
                    newCov = (Identity - K_j * H_j) * currentCov;

                    // update the state and covariance
                    raphael.updateStateVec(stateUpdate);
                    raphael.updateCov(newCov);
                }
                
                markerArrayFake_flag = false;
            }

            /**********
             * Publish a transform from world to map
             * *******/
            tf2::Quaternion worldMapQuater;
            worldMapQuater.setRPY(0.0, 0.0, 0.0);
            geometry_msgs::Quaternion worldMapQuat = tf2::toMsg(worldMapQuater);

            geometry_msgs::TransformStamped worldMapTrans;
            worldMapTrans.header.stamp = current_time;
            worldMapTrans.header.frame_id = world_frame_id;
            worldMapTrans.child_frame_id = map_frame_id;

            worldMapTrans.transform.translation.x = 0.0;
            worldMapTrans.transform.translation.y = 0.0;
            worldMapTrans.transform.translation.z = 0.0;
            worldMapTrans.transform.rotation = worldMapQuat;

            broadcaster.sendTransform(worldMapTrans);

            /**********
             * Pubish a transfrom from map to odom
             * *******/
            tf2::Quaternion mapOdomQuater;
            
            colvec currentStateVec = raphael.getStateVec();

            mapOdomQuater.setRPY(0.0, 0.0, currentStateVec(0) - ninjaTurtle.getTh());
            geometry_msgs::Quaternion mapOdomQuat = tf2::toMsg(mapOdomQuater);

            geometry_msgs::TransformStamped mapOdomTrans;
            mapOdomTrans.header.stamp = current_time;
            mapOdomTrans.header.frame_id = map_frame_id;
            mapOdomTrans.child_frame_id = odom_frame_id;

            mapOdomTrans.transform.translation.x = currentStateVec(1) - ninjaTurtle.getX();
            mapOdomTrans.transform.translation.y = currentStateVec(2) - ninjaTurtle.getY();
            mapOdomTrans.transform.translation.z = 0.0;
            mapOdomTrans.transform.rotation = mapOdomQuat;

            broadcaster.sendTransform(mapOdomTrans);

            /**********
             * Publish a transform from odom to body
             * *******/
            tf2::Quaternion odomBodyQuater;
            odomBodyQuater.setRPY(0.0, 0.0, ninjaTurtle.getTh());
            geometry_msgs::Quaternion odomBodyQuat = tf2::toMsg(odomBodyQuater);

            geometry_msgs::TransformStamped odomBodyTrans;
            odomBodyTrans.header.stamp = current_time;
            odomBodyTrans.header.frame_id = odom_frame_id;
            odomBodyTrans.child_frame_id = body_frame_id;

            odomBodyTrans.transform.translation.x = ninjaTurtle.getX();
            odomBodyTrans.transform.translation.y = ninjaTurtle.getY();
            odomBodyTrans.transform.translation.z = 0.0;
            odomBodyTrans.transform.rotation = odomBodyQuat;

            broadcaster.sendTransform(odomBodyTrans);

            /**********
             * Publish a nav_msgs/Path showing the trajectory of the robot according only to the wheel odometry
             * *******/
            geometry_msgs::PoseStamped odom_poseStamp;
            odom_path.header.stamp = current_time;
            odom_path.header.frame_id = world_frame_id;
            odom_poseStamp.pose.position.x = ninjaTurtle.getX();
            odom_poseStamp.pose.position.y = ninjaTurtle.getY();
            odom_poseStamp.pose.orientation.z = ninjaTurtle.getTh();

            odom_path.poses.push_back(odom_poseStamp);
            odomPath_pub.publish(odom_path);

            /*********
             * Publish a nav_msgs/Path showing the trajectory of the robot according to the slam algorithm
             * ******/
            geometry_msgs::PoseStamped slam_poseStamp;
            slam_path.header.stamp = current_time;
            slam_path.header.frame_id = world_frame_id;
            slam_poseStamp.pose.position.x = currentStateVec(1);
            slam_poseStamp.pose.position.y = currentStateVec(2);
            slam_poseStamp.pose.orientation.z = currentStateVec(0);

            slam_path.poses.push_back(slam_poseStamp);
            slamPath_pub.publish(slam_path);

            jointState_flag = false;
        }

        loop_rate.sleep();
    }
    return 0;
}

/// \brief callback function for subscriber to the sensor message
/// \param msg : the visualizaiton message
void sensorCallback(const visualization_msgs::MarkerArray array)
{
    marker_array = array;
    markerArray_flag = true;
}

void fakeSensorCallback(const visualization_msgs::MarkerArray array)
{
    marker_array_fake = array;
    markerArrayFake_flag = true;
}

/// \brief callback function for subscriber to joint state message
/// Sends an odometry message and broadcasts a tf transform to update the configuration of the robot
/// \param msg : the joint state message
void jointStateCallback(const sensor_msgs::JointState msg)
{
    joint_state_msg = msg;
    jointState_flag = true;
}

/// \brief setPose function for set_pose service
/// The service request provides the configuration of the robot
/// The location of the odometry is reset so the robot is at the requested configuration
/// \param req : The service request
/// \param res : The service reponse
/// \return true
bool setPose(rigid2d::set_pose::Request &req, rigid2d::set_pose::Response &res)
{
    using namespace rigid2d;

    /****************************
    * Configuration of the robot
    ****************************/
    double xNew = req.x;
    double yNew = req.y;
    double thNew = req.th;

    /****************************
    * Location of odometry reset so robot is at requested location
    * Replaces ninjaTurtle with a new configuration
    ****************************/
    ninjaTurtle = DiffDrive(wheelBase, wheelRad, xNew, yNew, thNew, 0.0, 0.0);

    return true;
}