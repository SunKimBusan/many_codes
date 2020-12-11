tmux kill-session
sleep 1
tmux new-session -d
tmux set -g mouse on
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -R
# tmux split-window -h
# tmux select-pane -R
tmux select-pane -t 0
tmux send "source ~/.bashrc" C-m
tmux send "export TURTLEBOT3_MODEL=burger" C-m
tmux send "roslaunch followbot_turtle followbot_turtle.launch" C-m
tmux select-pane -t 1
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_turtle" C-m
tmux send "cd src" C-m
tmux send "rosrun followbot_turtle clothes_pattern.py" C-m
tmux select-pane -t 2
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_turtle" C-m
tmux send "cd src" C-m
tmux send "rosrun followbot_turtle face_recog_raspberry.py" C-m

tmux select-layout even-horizontal
tmux attach-session -d
