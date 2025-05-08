#!/bin/bash

# launch mavros (multi_ros)
tmux new-session -d -s bundle -n mavros
tmux send-keys -t bundle:0 './lala_mavros.sh' C-m

while ! pgrep -x "mavros_node" > /dev/null; do
    sleep 1  # Check every second if roscore is running
done

# prepare offb pane
tmux split-window -h -t bundle:0
tmux send-keys -t bundle:0.1 'source ~/exp_ws/devel/setup.bash' C-m

# launch stream-rate setting
tmux new-window -t bundle:1 -n stream_rate
tmux send-keys -t bundle:1 './set_stream_rate.sh' C-m

# launch cam
tmux new-window -t bundle:2 -n cam
tmux send-keys -t bundle:2 './cam.sh' C-m

# launch odom
tmux new-window -t bundle:3 -n odom
tmux send-keys -t bundle:3 './odom.sh' C-m
tmux split-window -h -t bundle:3
tmux send-keys -t bundle:3.1 'rostopic echo /mavros/mocap/pose' C-m
tmux split-window -v -t bundle:3.1
tmux send-keys -t bundle:3.2 'rostopic echo /mavros/local_position/pose'

# launch dvl
tmux new-window -t bundle:4 -n dvl
tmux send-keys -t bundle:4 'echo "can do ./dvl.sh"' C-m

# attach to it
tmux attach -t bundle:0.1
