# robotont\_gazebo

[![Build Status](https://travis-ci.com/robotont/robotont_gazebo.svg?branch=melodic-devel)](https://travis-ci.com/github/robotont/robotont_gazebo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Dependencies

* [robotont_description](https://github.com/robotont/robotont_description)

* [robotont_nuc_description](https://github.com/robotont/robotont_nuc_description)

## Running the simulator

To run the simulator:

```bash
roslaunch robotont_gazebo gazebo.launch
```
The launch file has four arguments:

* model - chooses between a model with NUC and realsense and a model without them
    * default: robotont_gazebo_nuc 
    * options: robotont_gazebo_nuc, robotont_gazebo_basic, robotont_gazebo_lidar
* world - chooses which world to use
    * default: empty.world
    * options: empty.world, minimaze.world, bangbang.world, between.world, colors.world
* x_pos - chooses x coordinate of the world, controls where the robot will spawn, default: 0



For example, the following command will spawn the robot to a map called bangbang.world in position x=2 and the model that will be used is robotont_gazebo_nuc.
```bash
roslaunch robotont_gazebo gazebo.launch world:=$(rospack find robotont_gazebo)/worlds/bangbang.world model:=robotont_gazebo_nuc x_pos:=2 
```

To simply run the worlds without using the arguments:

*   ```bash
    roslaunch robotont_gazebo world_minimaze.launch
    ```
*   ```bash
    roslaunch robotont_gazebo world_bangbang.launch
    ```

*   ```bash
    roslaunch robotont_gazebo world_between.launch
    ```
*   ```bash
    roslaunch robotont_gazebo world_colors.launch
    ```
