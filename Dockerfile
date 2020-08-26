FROM osrf/ros:melodic-desktop

# Install some tools
RUN apt-get update && apt-get -y upgrade && apt-get install -y \
    python-rosdep \
    python-catkin-tools \
    python-vcstool \
  && rm -rf /var/lib/apt/lists/*

# Clone the source code
WORKDIR /sim_ws
COPY leo-erc.repos ./
RUN vcs import < leo-erc.repos

# Install dependencies
RUN apt-get update \
  && rosdep update \
  && rosdep install --from-paths src -iy \
  && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN catkin config --extend /opt/ros/melodic --install -i /opt/ros/leo-sim && catkin build

# Modify the entrypoint file
RUN sed -i "s|\$ROS_DISTRO|leo-sim|" /ros_entrypoint.sh

# Run launch file
CMD ["roslaunch", "leo_gazebo", "leo_marsyard.launch"]