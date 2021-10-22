# Nuturtlesim
This package contains a simulator for the turtlebot and its landmark-filled environment (tubes). This simulator simulates the kinematics of the robot and a sensor that detects the relative (x,y) positions of landmarks and their ids.

## Overview
The package contains the following nodes:
* `tube_world` - this node simulates the turtlebot's environment and the sensors used to detect landmarks. It tracks the ground truth location of the robot, maintains the positions of obstacles (landmarks and walls), detects collisions between the robot and the landmarks, and simulates a laser scanner using a point cloud.

The `tube_world` node maintains the ground truth positions of the obstacles relative to the `world` frame as shown below,

![real_tubes](img/real_tubes.png)

The node also maintains the positions of the obstacles relative to the turtlebot's location. The node calculates the distance between the turtlebot and the obstacle, and if it is within a certain range, then the landmark is visible. Otherwise, the landmark is no longer visible. See below in a small snippet where the turtlebot explores the environment,

![rel_tubes](img/rel_tubes.gif)

The node models the turtlebot and the obstacles as having a circle footprint. If these footprints collide with each other (meaning that the robot collides with an obstacle), then the turtlebot slips (with odometry updated) in the direction of the line tangent between the robot and the obstacle as seen below,

![collision](img/collision.gif)

Lastly, the node simulates lidar scanner data by publishing `sensor_msgs/LaserScan` messages. This is done by taking distance measurements between the turtle and any obstacles within the scanner's range. Circle-line intersection measurements were calculated such that the tube of radius `r` was intersected by the line defined by points 1 (the location of the robot relative to the tube) and 2 (the end point of the scanner's maximum range at a given angle). See below for what the lidar scanner looks like in `rviz`.

![lidar_scanner](img/lidar_scanner.gif)

## Example Usage
Open a terminal and run the following command
```
roslaunch nuturtlesim tube_world.launch
```
This will open an `rviz` simulation in which the robot is surrounded by tubes and four walls. Based on these markers, the simulation will send out LaserScan messages, which can be used in the ``` nuslam``` package.