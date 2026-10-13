#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# workspace overlay is present only once the packages are built
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

exec "$@"
