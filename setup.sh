git pull --recurse-submodules
git submodule update --init

rosdep update
sudo apt update
rosdep install --rosdistro ${ROS_DISTRO} --from-paths src -iy

catkin config --extend /opt/ros/${ROS_DISTRO}
catkin build