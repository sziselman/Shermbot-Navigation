# Nuturtle Robot
* A package that contians nodes that can be run on the turtlebot.

# Example Usage
Open a terminal and run the following command:
```
roscore
```
Open a new terminal and run the following commands:
```
ssh ubuntu@shermbot.local
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
Open a new terminal and run the following commands depending on whether you want to run follow_circle (true) or teleop_keyboard(false):
```
roslaunch nuturtle_robot odom_teleop.launch circle:=true
roslaunch nuturtle_robot odom_teleop.launch circle:=false
```
If you run the ``` follow_circle ``` node, it will automatically put you in an "Idle" state. To control the robot's movements, open a new terminal and run the following commands.
To have the robot drive in a counter-clockwise circle:
``` 
rosservice call /control ccw
```
To have the robot drive in a clockwise circle:
```
rosservice call /control cw
```
To have the robot stop driving:
```
rosservice call /control stop
```

# GIF Animations of the robot's movements
I currently can't upload my gifs as they are over 100MB and too large to upload to my git. Am working on compressing them down.
Also, I am still fixing my odometer node, as it currently says my x and y locations are in the hundreds of thousands.. which is incorrect.

Below is a .gif animation of the robot moving in backward/forward motion.
![](https://drive.google.com/uc?export=view&id=10TJfIkKhyZBhw4tlfTHuNQmEMD_IuyU_)
Below is a .gif animation of the robot rotating counter-clockwise/clockwise.
![](https://drive.google.com/uc?export=view&id=1iV1cexMgyEQOZ-HSeTIhQliZsY_ycmUM)
Below is a .gif animation of the robot driving in a counter-clockwise/clockwise circle.
![](https://drive.google.com/uc?export=view&id=1DG9hfBL64F_pzBvydPeIy8Hqr4bJBSTs)