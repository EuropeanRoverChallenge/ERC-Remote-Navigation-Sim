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
roslaunch leo_gazebo leo_marsyard.launch
```
For more info about available launch files and their arguments, visit the [leo_gazebo](https://github.com/LeoRover/leo_gazebo) repository.

To visualize the model in Rviz, type on another terminal session:
```
roslaunch leo_viz rviz.launch
```
Turn on the `Image` panel in Rviz to show the simulated camera images.

To control the Rover using a joystick, type:
```
roslaunch leo_teleop joy_teleop.launch
```
The command mapping was set for the Xbox 360 controller and looks like this:
| Xbox 360 controller       | Command                           |
|---------------------------|-----------------------------------|
| RB button                 | enable - hold it to send commands |
| Left joystick Up/Down     | linear velocity                   |
| Right Joystick Left/Right | angular velocity                  |
To modify it, you can edit the `joy_mapping.yaml` file inside the `leo_teleop` package.

## ROS API

The following topics and parameters are available on both the simulation and the real robot.

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover.  
    Only linear.x (m/s) and angular.z (r/s) are used.

### Published topics

* **`wheel_odom`** ([geometry_msgs/TwistStamped])

    Current linear and angular velocities of the robot estimated from wheel velocities.

* **`joint_states`** ([sensor_msgs/JointState])

    Current state of the joints. 

* **`camera/image_raw`** ([sensor_msgs/Image])

    Raw images from the hazard avoidance camera

* **`camera/image_raw/compressed`** ([sensor_msgs/CompressedImage])

    JPEG-compressed images from the hazard avoidance camera

* **`camera/camera_info`** ([sensor_msgs/CameraInfo])

    Meta information for the hazard avoidance camera (see [image_pipeline/CameraInfo]).

### Parameters set

* **`robot_description`** (type: `str`)

    The URDF model of the robot

[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[sensor_msgs/JointState]: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CompressedImage]: http://docs.ros.org/api/sensor_msgs/html/msg/CompressedImage.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
[image_pipeline/CameraInfo]: http://wiki.ros.org/image_pipeline/CameraInfo