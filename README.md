## 🤖 ROBOTICS LAB HOMEWORK3

This repository contains the ROS2 packages required for the homework 3 of the course. In the following the instructions to run the packages correctly.

##  BUILD AND SOURCE

Once you cloned the repository, build the packages.

```bash
colcon build
```

Then source 

```bash
source install/setup.bash 
```

## EXECUTE THE DETECTION OF THE BLUE SPHERE
Launch Gazebo
```bash
ros2 launch iiwa_bringup iiwa_sphere.launch.py command_interface:="velocity" robot_controller:="velocity_controller" initial_positions_file:="initial_sphere.yaml"

```

Run the code to detect the sphere correctly

```bash
ros2 run ros2_opencv ros2_opencv_node
```

Visualize the result on rqt

```bash
rqt 
```


## EXECUTE A POSITIONING TASK AND THE LOOK-AT-POINT TASK
First launch Gazebo 
```bash

ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" initial_positions_file:="initial_a.yaml"

```
On another terminal run
```bash

ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201

```
On another one, finally, run the node to execute the positioning task
```bash

ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p task:=positioning


```
You can stop the node when the trajectory is executed and then you can run
```bash
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p task:=look-at-point

```

## EXECUTE THE DYNAMIC VERSION OF THE VISION-BASED CONTROL
Launch the simulation in Gazebo
```bash

ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" initial_positions_file:="initial_b.yaml"


```
On another terminal run
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```bash

ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```


On another one execute the task
```bash
ros2 run ros2_kdl_package kdl_vision_effort

```









