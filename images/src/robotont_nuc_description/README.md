# robotont\_nuc\_description

Description package for ROBOTONT with an on-board NUC and RealSense D435i depth camera.

[![Build Status](https://travis-ci.org/robotont/robotont_nuc_description.svg?branch=melodic-devel)](https://travis-ci.org/robotont/robotont_nuc_description)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Prerequisites:
* Install Intel RealSense description:<br/>
```sudo apt install ros-melodic-realsense2-description```

Visualizing the robot model in RViz
* For the simulated robot:<br/>
```roslaunch robotont_nuc_description display_simulated_robot.launch```

* For the real robot:<br/>
```roslaunch robotont_nuc_description display_real_robot.launch```
