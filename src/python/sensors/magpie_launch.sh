#!/usr/bin/env bash
# The native Humble process shares the ROS graph schema used by the robot stack.
set -eo pipefail
unset PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD PYTHONHOME AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION CYCLONEDDS_HOME
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
export LD_LIBRARY_PATH=/opt/golem-humble-python/lib
source /opt/ros/humble/setup.bash
source /opt/golem-magpie-humble/setup.bash
export PYTHONHOME=/opt/golem-humble-python
export PYTHONPATH=/opt/golem-humble-python/lib/python3.10/site-packages:${PYTHONPATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Optional script override also lets CPU transport tests use exactly this runtime.
if [ "${1:-}" = --script ]; then
    shift
    exec /opt/golem-humble-python/bin/python3.10 "$@"
fi
exec /opt/golem-humble-python/bin/python3.10 "$(dirname "$0")/magpie_ros.py" "$1"
