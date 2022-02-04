# robotont\_demos
This repository is a ROS package that contains various demos showing the capabilities of the Robotont platform

[![Build Status](https://travis-ci.org/robotont/robotont_demos.svg?branch=melodic-devel)](https://travis-ci.org/robotont/robotont_demos)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Before you begin
To run the demos it is necessary to have a Robotont robot and a user PC with Ubuntu Linux and ROS Melodic installed.

There are two approaches to get the Robotont and PC into the same ROS environment. A common prerequisite for both methods is that the hosts are connected to the same network. In the following examples, we assume the Robotont and the PC having the following configuration:

| Machine  | Hostname   | IP-address    |
|----------|------------|---------------|
| Robotont | robotont-1 | 192.168.1.1   |
| PC       | laptop-1   | 192.168.1.101 |

### Method 1: Hostname based setup

In this configuration, the robot and PC query each other via hostnames. It means that both hosts need to have each other's names associated with IP addresses. These hostname <--> IP pairs are defined in the `/etc/hosts` file. Use your favorite text editor and make sure the following entries exist.

**/etc/hosts on Robotont on-board computer:**
```bash
127.0.1.1 robotont-1
192.168.1.101 laptop-1
```

**/etc/hosts on PC:**
```bash
127.0.1.1 laptop-1
192.168.1.1 robotont-1
```

Next, we need to tell the PC to look for a ROS Master on Robotont. We do that by modifying a special environment variable named `ROS_MASTER_URI`, which by default points to localhost.

**on PC**, open a terminal and enter:
```bash
export ROS_MASTER_URI=http://robotont-1:11311
```
Now all ROS nodes you run in this terminal will connect to the Master on the Robotont. Test it with e.g. `rosnode list`.
Note that the environment variable has to be set for each terminal window! To make it automatic, you can add the line to the end of the `.bashrc` file in the home directory of the PC:

```bash
echo 'export ROS_MASTER_URI=http://robotont-1:11311' >> ~/.bashrc
```

### Method 2: IP-address based setup
If you want to configure IP based communication there is no need to edit the hosts file. Instead, a `ROS_IP` environmental variable has to be set on both sides:

**on Robotont on-board computer:**
```bash
export ROS_IP=192.168.200.1
```

**on PC:**
```bash
export ROS_MASTER_URI=http://192.168.200.1:11311
export ROS_IP=192.168.200.101
```

Similarly to the hostname based setup, append the commands to `.bashrc` to set the variables automatically.


## 3D mapping
**Setup**<br/>

*You only need to run these commands if this is the first time you you run this demo with the current user PC or ROBOTONT on-board computer.*

**On Robotont on-board computer**, install ROS wrapper of RTAB-Map<br/>

```bash
sudo apt update
sudo apt install ros-melodic-rtabmap-ros
```

**Launching the demo**<br/>

**On Robotont on-board computer**, launch 3d_mapping.launch<br/>
```bash
roslaunch robotont_demos 3d_mapping.launch
```

**On PC**, launch 3d_mapping_display.launch to visualize the result<br/>

```bash
roslaunch robotont_demos 3d_mapping_display.launch
```

## 2D mapping
**Setup**<br/>

*You only need to run these commands if this is the first time you you run this demo with the current user PC or ROBOTONT on-board computer.*

**On Robotont on-board computer**, install the following packages:<br/>

```bash
sudo apt update
sudo apt install ros-melodic-depthimage-to-laserscan
sudo apt install ros-melodic-cartographer-ros
sudo apt install ros-melodic-move-base
```

**Launching the demo**<br/>

**On Robotont on-board computer**, launch 2d_slam.launch<br/>
```bash
roslaunch robotont_demos 2d_slam.launch
```

To change the mapping method to gmapping or hector_slam, change the mapping_method argument. For example, to use gmapping:

```bash
roslaunch robotont_demos 2d_slam.launch mapping_method:=gmapping
```

**On PC**, launch 2d_slam_display.launch to visualize the result<br/>
```bash
roslaunch robotont_demos 2d_slam_display.launch
```

## AR tracking

*You only need to run these commands if this is the first time you you run this demo with the current user PC or ROBOTONT on-board computer.*

**On Robotont on-board computer**, install ROS wrapper for alvar<br/>
```bash
sudo apt update
sudo apt install ros-melodic-ar-track-alvar
```

**Launching the demo**<br/>

**On Robotont on-board computer**, launch ar_follow_the_leader.launch (change 5 with the AR tag number you intend to follow)<br/>
```bash
roslaunch robotont_demos ar_follow_the_leader.launch marker_id:=5
```

**On PC**, launch ar_marker_display.launch to visualize the result<br/>
```bash
roslaunch robotont_demos ar_marker_display.launch
```
