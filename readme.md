# hex_arm_controller
- This package only contains the ROS node for sending control commands to xpkg_arm. To control a real arm, you must have the xpkg_arm package installed and running.
- This package supports both ros1 and ros2.

# Quick Start
## Perparing the environment
Before running the node, make sure that you have the xpkg_arm_msgs package installed.
```
wget -O hextool.bash https://ros.dl.hexmove.cn/ros.dl/hextool.bash && bash hextool.bash
```
Select the 'Force add hexmove apt source' option, the run `sudo apt update` to update the apt source.
Then, install the xpkg_arm_msgs package.
```
sudo apt install ros-noetic-xpkg-arm-msgs
```

## Run the node
To run the node, follow these steps:

1. Clone the repository into your catkin workspace
2. Build the package using `catkin_make` or `colcon build`
3. Source the setup.bash file in your workspace
4. Run the node using launch file:
```
roslaunch hex_arm_controller joint_forward.launch
```
or
```
ros2 launch hex_arm_controller joint_forward.launch.py
```

Then, you can echo the topic `/xtopic_arm/joints_cmd` to check the messages being published.

## How to use for your own project
- You can move the `utility` file to your own package and use it as `arm_controller.py` to build your own message.