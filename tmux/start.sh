#!/bin/bash

WORKSPACE_PATH=$(cd $(dirname $0)/../.. && pwd)
cd "$WORKSPACE_PATH"

#source /opt/ros/noetic/setup.bash
#source devel/setup.bash

tmuxinator start -p ./src/tmux/session.yml

if [ -z "$TMUX" ]; then
  tmux -L ros_socket attach -t ros_nodes
fi
