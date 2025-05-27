export ROS_HOSTNAME=192.168.2.2
export ROS_MASTER_URI=http://192.168.2.2:11311
export ROS_IP=192.168.2.2
rostopic pub -r 10 /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 1.0
    z: -0.150
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
