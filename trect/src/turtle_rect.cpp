/// \file turtle_rect.cpp 
/// \brief contains a node called turtle_rect
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     topic_name (topic_type): description of topic
/// SUBSCRIBES:
///     topic_name (topic_type): description of the topic
/// SERVICES:
///     service_name (service_type): description of the service
/// Roughly followed tutorials provided by turtlesim page on ros wiki
/// Used the following link as a source: https://cse.sc.edu/~jokane/agitr/agitr-letter-param.pdf


#include <ros/ros.h>
#include <trect/start.h>

#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/TeleportRelative.h"
#include "turtlesim/Color.h"
#include "std_srvs/Empty.h"

#include <sstream>
#include "std_msgs/String.h"

const double PI = 3.14159265359;


/************************************************************************
* Helper Function Declaration
************************************************************************/

bool start(trect::start::Request &req, trect::start::Response &res);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_msg);

static ros::Publisher pub;
static ros::Subscriber sub;
static ros::ServiceServer start_service;
static ros::ServiceClient start_client;
static ros::ServiceClient setPen_client;
static ros::ServiceClient teleAbs_client;
static ros::ServiceClient teleRel_client;
static ros::ServiceClient clear_client;

static turtlesim::PoseConstPtr turtle_pose;

enum State {Idle, bottomLine, rightLine, topLine, leftLine, Rotate};

State currentState, previousState;

static double width, height;

/************************************************************************
* Declare desired position of the turtle
************************************************************************/
double desiredX, desiredY, desiredTh;

/************************************************************************
* Main Function
************************************************************************/

int main(int argc, char* argv[])
{
    /**********************
    * Initialize the node & node handle
    **********************/
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle n;


    /**********************
    * Initialize variables
    **********************/
    double max_xdot, max_wdot, linVel, angVel;
    int frequency;

    geometry_msgs::Twist msg;

    /**********************
    * Reads parameters from parameter server
    **********************/
    n.getParam("max_xdot", max_xdot);
    n.getParam("max_wdot", max_wdot);
    n.getParam("frequency", frequency);


    linVel = max_xdot;
    angVel = max_wdot;


    /**********************
    * Initialize Publisher, Subscriber & Service
    **********************/
    pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", frequency);
    sub = n.subscribe("/turtle1/pose", 10, poseCallback);
    start_service = n.advertiseService("start", start);
    start_client = n.serviceClient<trect::start>("start");
    setPen_client = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    teleAbs_client = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    teleRel_client = n.serviceClient<turtlesim::TeleportRelative>("turtle1/teleport_relative");
    clear_client = n.serviceClient<std_srvs::Empty>("clear");
    
    
    ros::Rate loop_rate(frequency);    // ensures a somewhat-consistent publishing frequency

    /**********************
    * Log parameters as ROS_INFO messages
    **********************/
    ROS_INFO("max_xdot: %f max_wdot: %f frequency: %d", max_xdot, max_wdot, frequency);
    
    /********************************************
    * main while loop
    ********************************************/
    while(ros::ok())
    {
        ros::spinOnce();
        switch (currentState)
        {
            case Idle:
                msg.linear.x = 0;
                msg.angular.z = 0;
                pub.publish(msg);
                break;
            case bottomLine:
                msg.linear.x = linVel;
                msg.angular.z = 0;
                if (desiredX - turtle_pose->x < 0)
                {
                    desiredTh = PI/2;                // update the desired angle of the turtle

                    currentState = Rotate;
                    previousState = bottomLine;
                }
                pub.publish(msg);
                break;

            case rightLine:
                msg.linear.x = linVel;
                msg.angular.z = 0;
                if (desiredY - turtle_pose->y < 0)
                {
                    desiredTh = PI;

                    currentState = Rotate;
                    previousState = rightLine;
                }
                pub.publish(msg);
                break;

            case topLine:
                msg.linear.x = linVel;
                msg.angular.z = 0;

                if (desiredX - turtle_pose->x > 0)
                {
                    desiredTh = -PI/2;

                    currentState = Rotate;
                    previousState = topLine;
                }
                pub.publish(msg);
                break;

            case leftLine:
                msg.linear.x = linVel;
                msg.angular.z = 0;

                if (desiredY - turtle_pose->y > 0)
                {
                    desiredTh = 0;
                    
                    currentState = Idle;
                    previousState = leftLine;
                }
                pub.publish(msg);
                break;

            case Rotate:
                msg.linear.x = 0;
                msg.angular.z = angVel;

                if (previousState == bottomLine)
                {
                    if (fabs(desiredTh - turtle_pose->theta) < 0.01)
                    {
                        desiredY += height;
                        ROS_INFO("New desired y is %f", desiredY);
                        currentState = rightLine;
                        previousState = Rotate;
                    }
                }
                else if (previousState == rightLine)
                {
                    if (fabs(desiredTh - turtle_pose->theta) < 0.01)
                    {
                        desiredX -= width;
                        currentState = topLine;
                        previousState = Rotate;
                    }
                }
                else if (previousState == topLine)
                {
                    if (fabs(desiredTh - turtle_pose->theta) < 0.01)
                    {
                        desiredY -= height;
                        currentState = leftLine;
                        previousState = Rotate;
                    }
                }
                pub.publish(msg);
                break;
        }

    }
    
    return 0;
}

/************************************************************************
* Helper Function Declarations
************************************************************************/


void poseCallback(const turtlesim::Pose::ConstPtr & pose_msg)
{
    turtle_pose = pose_msg;
}

bool start(trect::start::Request &req, trect::start::Response &res)
{
    width = req.width;
    height = req.height;

    /***************************
    * Set the first desired location of the turtle
    ***************************/
    desiredX = req.x + req.width;
    desiredY = req.y;
    desiredTh = 0;

    ROS_INFO("The location is (%f, %f). The dimensions are (%f, %f)", (double) req.x, (double) req.y, (double) req.width, (double) req.height);

    /***************************
    * Set color of the pen to indigo
    ***************************/
    turtlesim::SetPen turtle_pen;
    turtlesim::TeleportAbsolute turtle_absPos;
    std_srvs::Empty empty;

    turtle_pen.request.r = 253;
    turtle_pen.request.b = 150;
    turtle_pen.request.g = 253;
    turtle_pen.request.width = 5;
    turtle_pen.request.off = 0;

    /***************************
    * Set color of the background of turtlesim
    ***************************/
    ros::param::set("sim/background_r", 255);
    ros::param::set("sim/background_g", 192);
    ros::param::set("sim/background_b", 203);
    ros::spinOnce();

    /***************************
    * Clear the background of the turtle simulator
    ***************************/
    setPen_client.call(turtle_pen);
    clear_client.call(empty);

    turtle_absPos.request.x = req.x;
    turtle_absPos.request.y = req.y;
    turtle_absPos.request.theta = 0;

    /***************************
    * Move turtle to designated start position
    ***************************/
    teleAbs_client.call(turtle_absPos);

    /***************************
    * Clear
    ***************************/
    clear_client.call(empty);

    /***************************
    * Have turtle draw the rectangle in indigo
    ***************************/
    turtle_absPos.request.x += width;
    teleAbs_client.call(turtle_absPos);

    turtle_absPos.request.y += height;
    teleAbs_client.call(turtle_absPos);

    turtle_absPos.request.x -= width;
    teleAbs_client.call(turtle_absPos);

    turtle_absPos.request.y -= height;
    teleAbs_client.call(turtle_absPos);

    /****************************
    * Change the color of the pen to lavender
    ****************************/
    turtle_pen.request.r = 182;
    turtle_pen.request.g = 104;
    turtle_pen.request.b = 182;
    setPen_client.call(turtle_pen);

    /***************************
    * Put turtle in bottom line state
    ***************************/
    currentState = bottomLine;
    return true;
}