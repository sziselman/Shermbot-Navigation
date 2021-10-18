# Turtle Rect
## Overview
A package that commands the turtle to follow a rectangular trajectory. The `trect` node keeps track of the turtle's position in the `turtlesim_node` using a subscriber. Depending on the position of the turtle, it will determine what linear and/or angular velocity it needs to publish to `cmd_vel` in order to follow its desired rectangular trajectory.

## Example Usage
Open one terminal and run the following command,
```
roslaunch trect trect.launch
```

## Configuration
In order to implement the `start` service, open another terminal and run the following command,
```
rosservice call /start "x: 1.0 y: 2.0 width: 3.0 height: 4.0"
```
Note that any values can be inputted for `x`, `y`, `width`, or `height`. Those listed above do not need to be used. Once the service has been called, the `turtlesim_node` should result in the following,

![Demonstration](<trect.gif>)