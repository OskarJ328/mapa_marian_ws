#!/bin/bash

nazwa_sesji="mapowanie"

tmux new-session -d -s "$nazwa_sesji"

tmux split-window -h -t "$nazwa_sesji":0

tmux split-window -v -t "$nazwa_sesji":0.0

tmux split-window -v -t "$nazwa_sesji":0.2

for i in 0 1 2 3; do
    tmux send-keys -t "$nazwa_sesji":0.$i 'cd ~/ros2_ws' Enter
    tmux send-keys -t "$nazwa_sesji":0.$i 'source install/setup.bash' Enter
done
tmux send-keys -t "$nazwa_sesji":0.0 'docker run -it --rm --net=host --device=/dev/ttyUSB0 microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0' Enter
tmux send-keys -t "$nazwa_sesji":0.1 'ros2 launch urg_node2 urg_node2.launch.py' Enter
tmux send-keys -t "$nazwa_sesji":0.2 'ros2 launch my_tf_broadcaster my_tf_launch.py' Enter
tmux send-keys -t "$nazwa_sesji":0.3 'ros2 launch slam_toolbox online_async_launch.py' Enter

tmux attach-session -t "$nazwa_sesji"