#!/bin/bash

# MIT License

# Copyright (c) 2023 Shunsuke Kimura

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

export ROS2_ALIASES=$BASH_SOURCE
export ROS_DISTRO=humble

function red  { echo -e "\033[31m$1\033[m"; }
function green { echo -e "\033[32m$1\033[m"; }
function cyan { echo -e "\033[36m$1\033[m"; }

if [ $# = 0 ]; then
  red "[ros2 aliases] Give at least one path as an argument."
  red "[Usage 1] source PATH_TO_CLONE/ros2_aliases.bash ROS_WORKSPACE"
  red "[Usage 2] source PATH_TO_CLONE/ros2_aliases.bash ROS_WORKSPACE COLCON_BUILD_CMD"
  red "[Usage 3] source PATH_TO_CLONE/ros2_aliases.bash CONFIG_FILE"
  return
fi

# config file load function
function load_config_yaml {
  local yaml_file=$1
  source "`dirname $ROS2_ALIASES`/yaml.sh"
  local yaml_string
  yaml_string="$(parse_yaml "$yaml_file")"
  eval "$(echo "$yaml_string" | sed 's/ROS2_ALIASES_ENVIRONMENT_VARIABLES_\(.*\)=\(.*\)/export \1=\2/g')"
  export ROS_WORKSPACE=$(eval echo "$ROS2_ALIASES_ROS_WORKSPACE")
  export COLCON_BUILD_CMD=$(eval echo "$ROS2_ALIASES_COLCON_BUILD_CMD")
  unset_variables "$yaml_string"
}

# arguments handling
case "$1" in
  *.yaml | *.yml )
    load_config_yaml "$1" > /dev/null
    ;;
  * ) 
    export ROS_WORKSPACE=$1
    if [ -n "$2" ]; then
      export COLCON_BUILD_CMD="$2"
    else
      export COLCON_BUILD_CMD="colcon build --symlink-install --parallel-workers $(nproc)"
    fi
    ;;
esac

# error check
if [ ! -d "$ROS_WORKSPACE/src" ]; then
  red "[ros2 aliases] No src directory in the workspace : $ROS_WORKSPACE"
  return
fi
if [[ $COLCON_BUILD_CMD != "colcon build "* ]]; then
  red "Invalid command for colcon build : $COLCON_BUILD_CMD"
  return
fi

# source other scripts
source "`dirname $ROS2_ALIASES`/ros2_utils.bash"
source /opt/ros/$ROS_DISTRO/setup.bash
local ws_setup_file=$ROS_WORKSPACE/install/setup.bash
if [ -e $ws_setup_file ]; then
  source $ws_setup_file
fi

# ros2 aliases help
function rahelp {
  green "--- change environments ---"
  echo "`cyan raload` : search and load config for ros2-aliases"
  echo "`cyan chws\ PATH_TO_WORKSPACE` : change ROS 2 workspace"
  echo "`cyan chcbc\ COLCON_BUILD_COMMAND` : change colcon build command with its arguments"
  echo "`cyan chrdi\ ROS_DOMAIN_ID` : change ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY"
  green "--- colcon build ---"
  echo "`cyan cb`    : colcon build"
  echo "`cyan cbp`   : colcon build with packages select"
  echo "`cyan cbcc`   : colcon build with clean cache"
  echo "`cyan cbcf`   : colcon build with clean first"
  green "--- roscd ---"
  echo "`cyan roscd` : cd to the selected package"
  green "--- ROS CLI ---"
  echo "`cyan rnlist` : ros2 node list"
  echo "`cyan rninfo` : ros2 node info"
  echo "`cyan rtlist` : ros2 topic list"
  echo "`cyan rtinfo` : ros2 topic info"
  echo "`cyan rtecho` : ros2 topic echo"
  echo "`cyan rplist` : ros2 param list"
  echo "`cyan rpget`  : ros2 param get"
  echo "`cyan rpset`  : ros2 param set"
  green "--- TF ---"
  echo "`cyan view_frames\ \(namespace\)` : ros2 run tf2_tools view_frames"
  echo "`cyan tf_echo\ \[source_frame\]\ \[target_frame\]\ \(namespace\)` : ros2 run tf2_ros tf2_echo"
  green "--- rosdep ---"
  echo "`cyan rosdep_install` : rosdep install"
  green "--- offical ---"
  echo "`cyan "ros2 -h"` : The Official help"
  green "--- current settings ---"
  echo "`cyan ROS_WORKSPACE` : "$ROS_WORKSPACE""
  echo "`cyan COLCON_BUILD_CMD` : "$COLCON_BUILD_CMD""
}

# ---change environments---
function raload {
  local CONFIG_FILE=`find ~ \( -path "$HOME/.config" -o -name "ros2_aliases.bash" \) -prune -o -type f \( -name "*.sh" -o -name "*.bash" -o -name "*.yaml" -o -name "*.yml" \) -exec grep -l "ROS2_ALIASES" {} + | fzf`
  if [ -n "$CONFIG_FILE" ]; then
    if [[ "$CONFIG_FILE" =~ \.sh$|\.bash$ ]]; then
      source $CONFIG_FILE
    elif [[ "$CONFIG_FILE" =~ \.yaml$|\.yml$ ]]; then
      load_config_yaml "$CONFIG_FILE" > /dev/null
    else
      red "*.sh, *.bash, *.yml, or *.yaml is required.*"
      return
    fi
    cyan "Load $CONFIG_FILE"
  fi
}

# change ROS 2 workspace
function chws {
  if [ -n "$1" ]; then
    cd $1 > /dev/null
  fi
  local arg=$(pwd)
  if [ ! -d "$arg/src" ]; then
    red "[ros2 aliases] No src directory in the workspace : $arg"
    return
  fi
  ROS_WORKSPACE=$arg
  echo "`cyan ROS_WORKSPACE` : "$ROS_WORKSPACE""
}

# change colcon build
function chcbc {
  if [ $# != 1 ]; then
    red "[Usage] chcbc COLCON_BUILD_CMD"
    echo "current COLCON_BUILD_CMD=\"`cyan "$COLCON_BUILD_CMD"`\""
    echo "default COLCON_BUILD_CMD=\"`cyan "colcon build --symlink-install --parallel-workers $(nproc)"`\""
    return
  fi
  source $ROS2_ALIASES "$ROS_WORKSPACE" "$1"
}

# change ROS_DOMAIN_ID
function chrdi {
  if [ $# != 1 ] || [ $1 -eq 0 ]; then
    export ROS_LOCALHOST_ONLY=1
    echo "ROS_LOCALHOST_ONLY=1"
  else
    export ROS_LOCALHOST_ONLY=0
    export ROS_DOMAIN_ID=$1
    echo "ROS_DOMAIN_ID=$1"
  fi
}

# ---colcon build---
function colcon_build_command_set {
  cd $ROS_WORKSPACE
  cyan "$2"
  $2
  source ./install/setup.bash
  history -s $1
  history -s $2
}
function cb {
  colcon_build_command_set "cb" "$COLCON_BUILD_CMD"
}
function cbp {
  if [ $# -eq 0 ]; then
    PKG=$(find $ROS_WORKSPACE/src -name "package.xml" -print0 | while IFS= read -r -d '' file; do grep -oP '(?<=<name>).*?(?=</name>)' "$file"; done | fzf)
    [[ -z "$PKG" ]] && return
    CMD="$COLCON_BUILD_CMD --packages-select $PKG"
  else
    CMD="$COLCON_BUILD_CMD --packages-select $@"
  fi
  colcon_build_command_set "cbp $@" "$CMD"
}
function cbcc {
  colcon_build_command_set "cbcc" "$COLCON_BUILD_CMD --cmake-clean-cache"
}
function cbcf {
  CMD="$COLCON_BUILD_CMD --cmake-clean-first"
  cyan $CMD
  read -p "Do you want to execute? (y:Yes/n:No): " yn
  case "$yn" in
    [yY]*);;
    *) return ;;
  esac
  colcon_build_command_set "cbcf" "$CMD"
}


# ---roscd---
function roscd {
  if [ $# -eq 1 ]; then
    PKG_DIR_NAME=$1
  else
    PKG_DIR_NAME=$(find $ROS_WORKSPACE/src -name "package.xml" -printf "%h\n" | awk -F/ '{print $NF}' | fzf)
    [[ -z "$PKG_DIR_NAME" ]] && return
    cyan "roscd $PKG_DIR_NAME"
  fi
  PKG_DIR=$(find $ROS_WORKSPACE/src -name $PKG_DIR_NAME | awk '{print length() ,$0}' | sort -n | awk '{ print  $2 }' | head -n 1)
  if [ -z $PKG_DIR ]; then
    red "$PKG_DIR_NAME : No such directory"
    return
  fi
  CMD="cd $PKG_DIR"
  $CMD
  history -s "roscd $PKG_DIR_NAME"
  history -s $CMD
}

# ---rosdep---
alias rosdep_install="cd $ROS_WORKSPACE && rosdep install --from-paths src --ignore-src -y"

# ---pkg---
alias rpkgexe="ros2 pkg executables"

# ---Pull request to ros2_utils---
# ROS 2 run
function rrun {
  if [ $# -eq 0 ]; then
    PKG_NAME=$(ros2 pkg list | fzf)
    [[ -z "$PKG_NAME" ]] && return
    history -s "rrun $PKG_NAME"
    rrun $PKG_NAME
  elif [ $# -eq 1 ]; then
    PKG_AND_EXE=$(ros2 pkg executables | grep $1 | fzf)
    [[ -z "$PKG_AND_EXE" ]] && return
    CMD="ros2 run $PKG_AND_EXE"
    echo "$CMD"
    $CMD
    history -s $CMD
  fi
}

# ros2 interface
function rishow {
  INTERFACE=$(ros2 interface list | fzf | sed 's/ //g')
  [[ -z "$INTERFACE" ]] && return
  CMD="ros2 interface show $INTERFACE"
  echo $CMD
  $CMD
  history -s $CMD
}
