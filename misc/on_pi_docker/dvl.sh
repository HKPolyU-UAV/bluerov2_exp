export ROS_HOSTNAME=192.168.2.2
export ROS_MASTER_URI=http://192.168.2.2:11311
export ROS_IP=192.168.2.2
source ~/exp_ws/devel/setup.bash
roslaunch waterlinked_a50_ros_driver launch_dvl.launch
