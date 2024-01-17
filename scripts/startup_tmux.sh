#!/bin/sh
#
echo "Startup takes some seconds, please stand by"

tmux new-session -d -s robot-learning-session &
sleep 1
tmux send-keys "roscore" C-m
sleep 3

tmux splitw -h -p 50
tmux selectp -t 0
tmux splitw -v -p 80
tmux send-keys "source devel/setup.bash && mon launch skiros2_skill_learning_demo robot.launch" C-m
# Give robot_bringup.launch some time
sleep 3

tmux selectp -t 2
tmux splitw -v -p 40
tmux selectp -t 2
tmux send-keys "source devel/setup.bash && mon launch skiros2_skill_learning_demo skiros.launch" C-m
sleep 3
tmux selectp -t 3

tmux send-keys "source devel/setup.bash" C-m
tmux send-keys "python3 src/SkiROS2_skill_learning_demo/src/learning.py"
tmux attach -t robot-learning-session