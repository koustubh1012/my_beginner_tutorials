# My_Beginner_Tutorials

## Overview
This project contains a simple package to demonstrate publisher and subscriber node in ROS2 humble.
Note: The output from cpp-lint is stored in the reports directory
## Author

Koustubh (koustubh@umd.edu)

## Prerequisites
Please make sure that you have the follwing setup as a prerequisite for building and running this package
- Ubuntu 22.04
- ROS2 Humble
- colcon
- clang tools

## Cloning and Building the package
1. Source ROS2 underlay
```bash
source /opt/ros/humble/setup.bash
```
2. Create new workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

```
3. Clone the repository 
```bash
git clone https://github.com/koustubh1012/my_beginner_tutorials.git
```

4. Resolve missing dependencies
```bash
cd ..
rosdep install -i --from-path src --rosdistro humble -y

```
5. Build the workspace with colcon bubild
```bash
colcon build
```

6. Source Overlay
```bash
source install/setup.bash
```

## Running the Nodes
1. Run the talker node
```bash
ros2 run beginner_tutorials talker
```
2. Run the listener node

Open another terminal and run the following commands
```bash
# Source underlay
source /opt/ros/humble/setup.bash

# Source overlay
cd ~/ros2_ws
source install/setup.bash

# Run the Subscriber node
ros2 run beginner_tutorials listener
```

##
