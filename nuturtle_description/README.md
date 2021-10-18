# Nuturtle Description
This package adapts the model of the TurtleBot3 for the needs of this course and displays the robot in `rviz`. Use the following command to see the robot in rviz:
```
roslaunch nuturtle_description load.launch
```
This package contains a file called `diff_params.yaml` where the `wheel_base`and `wheel_radius` can be adjusted in the `turtlebot3_burger.urdf.xacro` file. Changing `wheel_radius` changes the collision geometry of the wheels and changing `wheel_base` changes the distance between the wheels.