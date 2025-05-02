# This is still under work

Tested with Livox Mid-360, Ubuntu 22.04. ROS2 and robotpy 2025

# todo

Launch file for NTbridge and ros driver

## Prerequisites
*  [Ubuntu](http://ubuntu.com) (tested on 22.04 )
*  [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 
*  Eigen
*  Ceres Solver
*  PCL
   ```
   sudo apt-get install libeigen3-dev libpcl-dev libceres-dev
   ```
*  [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
*  [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
*  Suitesparse
   ```
   sudo apt-get install libsuitesparse-dev
   ```
*  [Robotpy](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html)
*  [pyntcore](https://pypi.org/project/pyntcore/)

## Compilation
```
cd ~/catkin_ws/
git clone https://github.com/sikaxn/livox_lio_FRC
cd ..
colcon build
```
## Setup static IP for Ubuntu machine and Livox Lidar

Use LivoxViewer to set IP for Lidar.

## Setup config file for Lio-Livox

Go to ```\livox_ros_driver2\config\MID360_config.json```

set ```host_net_info``` to your Ubuntu IP address and set ```lidar_configs``` to your lidar IP


## Setup roboRIO IP address

open NTBridge.py and edit ip address

## Launch during test

I will try combine these into 1 launch file.

```
source install/setup.bash
ros2 launch livox_ros_driver2 msg_NMID360_launch.py
```

```
source install/setup.bash
ros2 launch lio_livox launch.py
```

```
python3 src/NTBridge2/NTBridge.py
```
