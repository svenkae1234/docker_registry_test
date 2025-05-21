#!/bin/bash

# Source ROS distro environment
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Source the local catkin workspace
if [ -f "${ROS_WS}/install/setup.bash" ]; then
  source "${ROS_WS}/install/setup.bash"
fi

# Function to build the workspace and then reload the history
cb() {
    cd ${ROS_WS}

    # Check if a parameter is passed
    if [ -z "$1" ]; then
        # No parameter passed, build up to specific package
        colcon build --mixin release ccache
    else
        # Parameter(s) passed, build only selected packages
        colcon build --packages-select "$@" --mixin release ccache
    fi
    
    history -a  # Append new history lines to the history file
    exec bash  # Start a new bash session
}

# Function to clean the workspace
cc() {
    unset AMENT_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
      source "/opt/ros/$ROS_DISTRO/setup.bash"
    fi
    
    cd ${ROS_WS}
    rm -r build install log
}

exec "$@"