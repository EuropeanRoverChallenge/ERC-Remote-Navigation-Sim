GAZEBO="false"
RVIZ="false"
TELEOP="false"

print_help()
{
    echo "LEO Rover Run script"
    echo 
    echo "Example:"
    echo "  ./run.sh -g"
    echo 
    echo "Note: Script can only turn on one option at a time."
    echo 
    echo "Options:"
    echo "-h or --help"
    echo "  Help information"
    echo 
    echo "-g or --gazebo"
    echo "  Run Gazebo simulation of LEO Rover"
    echo 
    echo "-r or --rviz"
    echo "  Run rViz simulation of LEO Rover."
    echo "Note that Gazebo needs to be already running for rVziz to work correctly."
    echo 
    echo "-tp or --teleop"
    echo "  Run tele-operation of LEO Rover using keyboard"
    echo 
}

launch_gazebo()
{
    catkin config --extend /opt/ros/${ROS_DISTRO}
    catkin build
    source devel/setup.bash

    roslaunch leo_erc_gazebo leo_marsyard.launch
}

launch_rviz(){
    roslaunch leo_erc_viz rviz.launch
}

launch_teleop(){
    rosrun leo_erc_teleop key_teleop
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      print_help
      shift
      ;;
    -g|--gazebo)
      GAZEBO="true"
      shift
      ;;
    -r|--rviz)
      RVIZ="true"
      shift
      ;;
    -t|--teleop)
      TELEOP="true"
      shift
      ;;
    -*|--*)
      echo "Unknown option $1"
      print_help
      exit
      ;;
  esac
done
shift "$(($OPTIND -1))"

if [ "$GAZEBO" == "true" ]; then
    launch_gazebo
elif [ "$RVIZ" == "true" ]; then
    launch_rviz
elif [ "$TELEOP" == "true" ]; then
    launch_teleop
else 
    print_help
fi

