export ROS_HOSTNAME=192.168.2.2
export ROS_MASTER_URI=http://192.168.2.2:11311
export ROS_IP=192.168.2.2
rostopic pub -r 10 /mavros/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped \
"{header: {frame_id: 'map'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.1}}}"
