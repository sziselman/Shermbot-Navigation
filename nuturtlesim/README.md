# Nuturtlesim
* A package that simulates the turtlebot's environment with tubes (landmarks)

# Example Usage
Open a terminal and run the following command
```
roslaunch nuturtlesim tube_world.launch
```
This will open a simulation in which the robot is surrounded by tubes and four walls. Based on these markers, the simulation will send out LaserScan messages, which can be used in the ``` nuslam``` package.