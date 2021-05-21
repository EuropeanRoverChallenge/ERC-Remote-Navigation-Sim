# ERC Remote Navigation Simulation

This repository provides a Gazebo simulation of the Navigation and Science Task for the ERC Remote competition. \
For the dockerized version, skip to the [Using Docker](#using-docker) section.

## Table of Contents
1. [Prerequisites](#prerequisites)
1. [Building](#building)
1. [Updating](#updating)
1. [Launching](#launching)
1. [Using Docker](#using-docker)
1. [ROS API](#ros-api)
    1. [Subscribed Topics](#subscribed-topics)
    1. [Published Topics](#published-topics)
    1. [Services](#services)
    1. [Parameters Set](#parameters-set)
1. [Troubleshooting](#troubleshooting)

## Prerequisites

The simulation requires ROS (Robot Operating System) and was mainly developed and tested with these distributions:
 - [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/) on [Ubuntu 18.04 Bionic Beaver](https://releases.ubuntu.com/18.04/)
 - [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/)

Other distributions might not work properly so it is recommended to use one of these two setups. \
The guide will refer to the distribution of your choice (for example, `melodic` or `noetic`) as `<distro>`.

The rest of the tools used in this guide can be installed with apt:
```sh
# if using ROS Melodic
sudo apt install python-rosdep python-catkin-tools
# if using ROS Noetic
sudo apt install python3-rosdep python3-catkin-tools
```

## Building
This repository uses git submodules to link external repositories that contain the ROS packages. \
When cloning this repository, add the `--recurse-submodules` flag to recursively pull the submodules:
```
git clone --recurse-submodules https://github.com/EuropeanRoverChallenge/ERC-Remote-Navigation-Sim.git
```
or if you have already cloned the repository without this option, clone the submodules using:
```sh
git submodule update --init
```
Use the `rosdep` tool to install any missing dependencies. If you are running `rosdep` for the first time, you might have to run:
```sh
sudo rosdep init
```
first. Then, to install the dependencies, type:
```sh
rosdep update
sudo apt update
rosdep install --rosdistro <distro> --from-paths src -iy
```
Now, use the `catkin` tool to build the workspace:
```sh
catkin config --extend /opt/ros/<distro>
catkin build
```

## Updating
To pull the newest commits and recursively update the submodules, simply type:
```sh
git pull --recurse-submodules
```

If you have already pulled the new commits without the `--recurse-submodules` flag, you can simply update the submodules:
```sh
git submodule update --init
```

Make sure you have installed any additional dependencies the new versions of the packages may have pulled by running the `rosdep install` command again:
```
rosdep install --rosdistro <distro> --from-paths src -iy
```

And rebuild the workspace:
```
catkin build
```

## Launching

Make sure you source the devel space on each terminal session you want to use the simulation on:
```
source devel/setup.bash
```

To start the simulation and gazebo GUI, type:
```
roslaunch leo_erc_gazebo leo_marsyard.launch
```

To visualize the model in Rviz, type on another terminal session:
```
roslaunch leo_erc_viz rviz.launch
```

Turn on the `Image` panel in Rviz to show the simulated camera images.

To test teleoperation with a keyboard, you can run the `key_teleop` node:
```
rosrun leo_erc_teleop key_teleop
```

To control the Rover using a joystick, type:
```
roslaunch leo_erc_teleop joy_teleop.launch
```

The command mapping was set for the Xbox 360 controller and looks like this:
| Xbox 360 controller       | Command                           |
|---------------------------|-----------------------------------|
| RB button                 | enable - hold it to send commands |
| Left joystick Up/Down     | linear velocity                   |
| Right Joystick Left/Right | angular velocity                  |

To modify it, you can edit the `joy_mapping.yaml` file inside the `leo_erc_teleop` package.

## Using Docker

---
**NOTE**

All of the commands in this section should be executed as the `root` user, unless you have configured docker to be [managable as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/).

---

Make sure the [Docker Engine](https://docs.docker.com/engine/install/#server) is installed and the `docker` service is running:
```
systemctl start docker
```

Then, either pull the newest prebuilt Docker image:
```
docker pull ghcr.io/europeanroverchallenge/erc-remote-navigation-sim:latest
docker tag ghcr.io/europeanroverchallenge/erc-remote-navigation-sim:latest erc_navigation_sim
```

or build the image yourself:
```
docker build -t erc_navigation_sim .
```

Permit the root user to connect to X window display:
```
xhost +local:root
```

Start the docker container:
```
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name erc_sim erc_navigation_sim
```

If you want the simulation to be able to communicate with ROS nodes running on the host or another docker container, add `--net=host` flag:
```
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name erc_sim --net=host erc_navigation_sim
```

Gazebo may not run or work really slow without the GPU acceleration. \
If you are running the system with an integrated AMD/Intel Graphics card, try adding `--device=/dev/dri` flag:
```
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name erc_sim --device=/dev/dri erc_navigation_sim
```

To use an Nvidia card, you need to have proprietary drivers installed, as well as the [Nvidia Container Toolkit](https://github.com/NVIDIA/nvidia-docker). \
Add the `--gpus all` flag and set `NVIDIA_DRIVER_CAPABILITIES` variable to `all`:
```
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name erc_sim --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all erc_navigation_sim
```

To use the other ROS packages, start `bash` inside the running container:
```
docker exec -it erc_sim /ros_entrypoint.sh bash
```

and use the examples from the [Launching](#launching) section.

## ROS API

This section describes ROS topics, services and parameters that are available on both the simulation and the real robot.

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover.  
    Only linear.x (m/s) and angular.z (r/s) are used.

* **`zed2/reset_odometry`** ([std_msgs/Empty])

    Resets the odometry published on the `zed2/odom` topic.

* **`probe_deployment_unit/drop`** ([std_msgs/Empty])

    Spawn probe model.

### Published topics

* **`wheel_odom`** ([geometry_msgs/TwistStamped])

    Current linear and angular velocities of the robot estimated from wheel velocities.

* **`wheel_pose`** ([geometry_msgs/PoseStamped])

    A 2D pose of the robot estimated from the wheel velocities.

* **`joint_states`** ([sensor_msgs/JointState])

    Current state of the wheel joints.

* **`camera/image_raw`** ([sensor_msgs/Image])

    Unrectified images from the hazard avoidance camera.

* **`camera/image_raw/compressed`** ([sensor_msgs/CompressedImage])

    JPEG-compressed images from the hazard avoidance camera.

* **`camera/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the hazard avoidance camera (see [image_pipeline/CameraInfo]).

* **`zed2/left_raw/image_raw_color`** ([sensor_msgs/Image])

    Unrectified color images from the left ZED2 camera.

* **`zed2/left_raw/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the left ZED2 unrectified camera.

* **`zed2/left/image_rect_color`** ([sensor_msgs/Image])

    Rectified color images from the left ZED2 camera.

* **`zed2/left/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the left ZED2 camera.

* **`zed2/right_raw/image_raw_color`** ([sensor_msgs/Image])

    Unrectified color images from the right ZED2 camera.

* **`zed2/right_raw/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the right ZED2 unrectified camera.

* **`zed2/right/image_rect_color`** ([sensor_msgs/Image])

    Rectified color images from the right ZED2 camera.

* **`zed2/right/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the right ZED2 camera.

* **`zed2/depth/depth_registered`** ([sensor_msgs/Image])

    Depth map image registered on left ZED2 camera image.

* **`zed2/depth/camera_info`** ([sensor_msgs/CameraInfo])

    Depth camera calibration data.

* **`zed2/point_cloud/cloud_registered`** ([sensor_msgs/PointCloud2])

    Registered color point cloud.

* **`zed2/imu/data`** ([sensor_msgs/Imu])

    Accelerometer, gyroscope, and orientation data from the ZED2 IMU.

* **`zed2/odom`** ([nav_msgs/Odometry])

    Estimated ZED2 Camera position and orientation in free space relative to the Odometry frame (visual-inertial odometry).

* **`ground_truth`** ([nav_msgs/Odometry]) **(Only in the simulation)**

    The actual position of the robot on the terrain. It may be useful for validating performance of a global localization solution.

* **`probe_deployment_unit/probes_dropped`** ([std_msgs/UInt8])

    The actual number of probes dropped. 

### Services

* **`core2/reset_odometry`** ([std_srvs/Trigger])

    Resets the pose published on the `wheel_pose` topic.

* **`probe_deployment_unit/home`** ([std_srvs/Trigger])

    Resets the PDU. In the simulation, it removes dropped probe models and resets the counter.
    **Warning:** Don't use this on the real robot during the competition.

### Parameters set

* **`robot_description`** (type: `str`)

    The URDF model of the robot.

* **`probe_description`** (type: `str`) **(Only in the simulation)**

    The URDF model of the probe.

* **`pdu_node/probe_spawn_translation/x | y | z`** (type: `float`) **(Only in the simulation)**

    Probe spawn point translation from the base_footprint frame.

[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[geometry_msgs/PoseStamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
[sensor_msgs/JointState]: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CompressedImage]: http://docs.ros.org/api/sensor_msgs/html/msg/CompressedImage.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
[image_pipeline/CameraInfo]: http://wiki.ros.org/image_pipeline/CameraInfo
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[nav_msgs/Odometry]: http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
[std_msgs/Empty]: http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html
[std_msgs/UInt8]: http://docs.ros.org/en/melodic/api/std_msgs/html/msg/UInt8.html
[std_srvs/Trigger]: http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html

## Troubleshooting

### D-Bus error
The D-Bus error may occure while trying to launch gazebo inside the docker container. The easiest way to solve the problem is to use the `--privileged` flag to give extended privileges to this container, for example:
```
docker run --rm --net=host -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --privileged --name erc_sim erc_navigation_sim
```
