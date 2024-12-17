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
sudo apt install ros-<ros_version>-xpkg-arm-msgs
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

## TO control the arm
### Config the CAN adapter
To control the robotic arm, you need to connect it via a "USB-to-CAN" adapter and configure the udev rules.
1. Copy the Rules File
Copy the scripts/99-hex-can.rules file to the /etc/udev/rules.d/ directory.
2. Check the Device ID
In the rules file, ensure that the following line matches your device ID:
```
ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f"
```
For example, if you run the lsusb command, you should see something like: `Bus 003 Device 011: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter`
If the device ID is not 1d50:606f, modify the rules file to match your device's ID.
3. Reload the udev Rules
After editing the rules file, reload the udev rules by running the following command:
```
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Once done, plug in your USB CAN adapter and use the `ip a` command to check if hexcan0 and vcan0 interfaces appear.

### Launch the node
1. Start CAN Adapter Setup
```
roslaunch hex_arm_controller can_setup.launch
```
or
```
ros2 launch hex_arm_controller can_setup.launch.py
```

The output should be something like:
```
hexcan0 open success  fd: 3
vcan0 open success  fd: 4
driver 6 online
driver 1 online
driver 2 online
driver 3 online
driver 4 online
driver 5 online
```
2. Launch the Arm Configuration
Then, run the xpkg_arm package:
```
roslaunch xpkg_arm arm_setup.launch
```
or
```
ros2 launch xpkg_arm arm_setup.launch.py
```
Now, you can control the arm by publishing the joints_cmd messages to the topic `/xtopic_arm/joints_cmd`.
3. Run a Simple Example
```
roslaunch hex_arm_controller joint_forward.launch
```
or
```
ros2 launch hex_arm_controller joint_forward.launch.py
```

## How to use for your own project
- You can move the `utility` file to your own package and use it as `arm_controller.py` to build your own message.