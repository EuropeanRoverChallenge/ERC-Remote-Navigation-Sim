# ERC Simulation Workspace

This repository aims to provide instructions on how to install and use the Leo Rover simulation environment for the ERC competitions.

## Requirements

The simulation is mainly developed and tested on [Ubuntu 18.04 Bionic Beaver](https://releases.ubuntu.com/18.04/) with [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu), so it is a recommended setup.

The rest of the tools used in this guide can be installed with apt:
```
sudo apt install python-rosdep python-catkin-tools python-vcstool
```

## Building

Use the `vcstool` tool to clone the required packages:
```
vcs import < leo-erc.repos
```
Use the `rosdep` tool to install any missing dependencies. If you are running `rosdep` for the first time, you might have to run:
```
sudo rosdep init
```
first. Then, to install the dependencies, type:
```
rosdep update
sudo apt update
rosdep install --rosdistro melodic --from-paths src -iy
```
Now, use the `catkin` tool to build the workspace:
```
catkin config --extend /opt/ros/melodic
catkin build
```

## Updating

The list of the repositories may change, so make sure you have pulled the latest commit:
```
git pull
```
Then, repeat the steps described at the `Building` section.

## Launching

Make sure you source the devel space on each terminal session you want to use the simulation on:
```
source devel/setup.bash
```
To start the simulation and gazebo GUI, type:
```
roslaunch leo_gazebo leo_gazebo.launch
```
For more info about available launch files and their arguments, visit the [leo_gazebo](https://github.com/LeoRover/leo_gazebo) repository.

To visualize the model in Rviz, type on another terminal session:
```
roslaunch leo_viz rviz.launch
```
Turn on the `Image` panel in Rviz to show the simulated camera images.
