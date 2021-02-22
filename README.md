# ERC Simulation Workspace

This repository aims to provide instructions on how to install and use the Leo Rover simulation environment for the ERC competitions.

## Requirements

The simulation is mainly developed and tested on [Ubuntu 18.04 Bionic Beaver](https://releases.ubuntu.com/18.04/) with [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu), so it is a recommended setup. 

The rest of the tools used in this guide can be installed with apt:
```
sudo apt install python-rosdep python-catkin-tools python-vcstool
```

There is also a dockerized version which should work on most Linux distributions running X Window Server (See [Using Docker](#using-docker) section).

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
Use `vcstool` tool to clone any new repositories:
```
vcs import < leo-erc.repos
```
And pull the new commits on the already cloned ones:
```
vcs pull src
```
Then, make sure you have all the dependencies installed:
```
rosdep install --rosdistro melodic --from-paths src -iy
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

    Accelerometer, gyroscope, and orientation data from ZED2 IMU.

### Parameters set

* **`robot_description`** (type: `str`)

    The URDF model of the robot.

## Using Docker

---
**NOTE**

The commands in this section should be executed as the `root` user, unless you have configured docker to be [managable as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/).

---

Make sure the [Docker Engine](https://docs.docker.com/engine/install/#server) is installed and the `docker` service is running:
```
systemctl start docker
```
Build the docker image by executing:
```
docker build -t erc_sim .
```
Permit the root user to connect to X window display:
```
xhost +local:root
```
Start the docker container:
```
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name erc_sim erc_sim
```
If you want the simulation to be able to communicate with ROS nodes running on the host or another docker container, add `--net=host` flag:
```
docker run --rm --net=host -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name erc_sim  erc_sim
```
Gazebo can work really slow without the GPU acceleration. \
If you are running the system on an integrated AMD/Intel Graphics card, try adding `--device=/dev/dri` flag:
```
docker run --rm --net=host --device=/dev/dri -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --name erc_sim  erc_sim
```
To use an Nvidia card, you need to have proprietary drivers installed, as well as the [Nvidia Container Toolkit](https://github.com/NVIDIA/nvidia-docker). \
Add the `--gpus all` flag and set `NVIDIA_DRIVER_CAPABILITIES` variable to `all`:
```
docker run --rm --net=host -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --name erc_sim erc_sim
```
To start any other ROS nodes inside the container, type:
```
docker exec -it erc_sim /ros_entrypoint.sh <COMMAND>
```
For example:
```
docker exec -it erc_sim /ros_entrypoint.sh roslaunch leo_viz rviz.launch
```
To update the docker image, you need to rebuild it with `--no-cache` option:
```
docker build --no-cache -t erc_sim .
```
## Troubleshooting

---
* ### D-Bus error
The D-Bus error may occure while trying to launch gazebo. To solve the problem use `--privileged` flag, it gives extended privileges to this container.
```
docker run --rm --net=host -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --privileged --name erc_sim erc_sim
```

[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[sensor_msgs/JointState]: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CompressedImage]: http://docs.ros.org/api/sensor_msgs/html/msg/CompressedImage.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
[image_pipeline/CameraInfo]: http://wiki.ros.org/image_pipeline/CameraInfo
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html