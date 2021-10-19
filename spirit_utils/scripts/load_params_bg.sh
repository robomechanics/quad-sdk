#!/bin/bash
tmux new-session -d -s load-params-session
tmux send-keys 'roslaunch spirit_utils load_params.launch' C-m
tmux detach -s load-params-session
# tmux kill-session -t load-params-session # <--- command to kill session