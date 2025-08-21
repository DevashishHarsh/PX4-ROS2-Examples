# PX4 ROS2 Workspace
This repository contains examples and all the required files to be used for controlling the drone and also to use my other repo Multi-Drone-PX4-RL. 

## Content
- Example files to launch in Gazebo with PX4 x500 Drones ( controlled with ROS2 Topics )
- px4_msgs and px4_ros_com files to be used by other repositories


## Installation for PX4-Autopilot
This installation follows the latest PX4-Autopilot from this website [PX4 Installation Guide](https://docs.px4.io/main/en/dev_setup/building_px4).
> Make sure to use "git checkout 38d67f5" as I have only worked with that specific release and it works for me.

## Installation for ROS2
You can follow the official ROS2 guide for installing it on your linux system using [ROS2 Installation Guide](https://docs.ros.org/en/jazzy/Installation.html).

## Installation for Micro XRCE-DDS
```
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
> If you get fastdds version error, refer to this for [solution](https://github.com/PX4/PX4-Autopilot/issues/24477)

