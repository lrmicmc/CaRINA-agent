#!/bin/bash

set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"
source "/workspace/team_code/catkin_ws/devel/setup.bash"

exec "$@"
