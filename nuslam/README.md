# NuSlam
* A package that contains nodes to implement EKF SLAM and Landmark Detection

# Example Usage
Open a terminal and run the following command to have sensor readings from the fake sensor (tube world):
```
roslaunch nuslam slam.launch real:=true
```
Open a terminal and run the following command to have sensor readings from the lidar (real) sensor (landmarks):
```
roslaunch nuslam slam.launch real:=false
```

# Notes
Although mostly everything looks fine, I believe there are still some small bugs to fix. I plan to make changes to complete data association.