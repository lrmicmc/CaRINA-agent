#!/bin/bash
set -e

source $TEAM_CODE_ROOT/catkin_ws/devel/setup.bash
source "${HOME}/agent_sources.sh"

exec "$@"
