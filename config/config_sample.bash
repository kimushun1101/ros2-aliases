# ros2-aliases
# Please refer to https://github.com/kimushun1101/ros2-aliases
ROS_WORKSPACE="$HOME/ros2_ws" # In the case of describing the full path
# ROS_WORKSPACE=`dirname $BASH_SOURCE`  # This file is place on your ROS_WORKSPACE
# ROS_WORKSPACE="`dirname $BASH_SOURCE`/.." # This file is place on the sub directory of your ROS_WORKSPACE
COLCON_BUILD_CMD="colcon build --symlink-install --parallel-workers $(nproc)"
if [ -n "$ROS2_ALIASES" ]; then
  source $ROS2_ALIASES $ROS_WORKSPACE "$COLCON_BUILD_CMD"
# else
#   echo -e "\033[33mIf you want to activate ros2-aliases, refer to https://github.com/kimushun1101/ros2-aliases\033[m"
fi
# the end of ros2-aliases

# Configuration Examples
# ADDITION_VARIABLES
export MY_ENV_1=1
export MY_ENV_2=10

# ROS_VARIABLES
export ROS_DOMAIN_ID=0
export ROS_OST_ONLY=1
export ROS_LOG_DIR="~/.ros/log"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# Aliases Examples
alias turtlesim="ros2 run turtlesim turtlesim_node"
alias teleopkey="ros2 run turtlesim turtle_teleop_key"
