export ROS_HOSTNAME=
rostopic pub -r 10 /mavros/setpoint_raw/local mavros_msgs/PositionTarget "header:
  frame_id: 'map'
  stamp: {secs: 0, nsecs: 0}
coordinate_frame: 1  # MAV_FRAME_LOCAL_NED
type_mask: 0b100111111000  # Ignore velocity, acceleration, yaw, yaw_rate; use position only
position:
  x: 1.0
  y: 1.0
  z: -0.1"
