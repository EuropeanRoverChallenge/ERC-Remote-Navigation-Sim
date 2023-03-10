catkin config --extend /opt/ros/${ROS_DISTRO}
catkin build

source devel/setup.bash

launch_gazebo()
{
    roslaunch leo_erc_gazebo leo_marsyard.launch
}

launch_rviz(){
    roslaunch leo_erc_viz rviz.launch
}

launch_gazebo & 

wait
