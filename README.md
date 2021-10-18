# Shermbot Navigation Project Winter 2021
Maintained by Sarah Ziselman for ME 495 Sensing, Navigating, & Machine Learning for Robotics. This project covers a broad range of areas in robotic software development using the TurtleBot3 burger robot including writing C++ libraries, unit testing, building a simulator, landmark detection, perception, and Extended Kalman Filter SLAM.

# Package List
This repository consists of the following ROS packages:
* `nuturtle_description` - A package that will be used to display a model of the turtlebot3 robot in rviz.
* `rigid2d` - A package containing several libraries used for performing 2D rigid body transformations and differential drive odometry updates. Also contains nodes that simulate a fake turtlebot and publish its corresponding `Odometry` messages.
* `trect` - A package that commands a turtle to follow a rectangular trajectory in the `turtlesim_node`.
* `nuturtlesim` - A package that contains a simulator for the turtlebot and its environment using `rviz`. It provides a space to simulate obstacles, paths, sensors, point clouds, and other desired information.
* `nuslam` - A package that contains several libraries used for circle fitting, landmark detection, and Extended Kalman Filter SLAM. Also contains nodes that perform EKF SLAM updates and are visualized in the `nuturtlesim` node.