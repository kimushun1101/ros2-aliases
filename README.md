# ros2-aliases
Collection of functions and aliases for ROS2 development

![](https://github.com/tonynajjar/ros2-aliases/blob/main/usage.gif)

# Prerequisites

- [fzf](https://github.com/junegunn/fzf#installation)
  For Ubuntu simply: 
  ```
  sudo apt install fzf
  ```
  For more install options refer to the documentation

- Bash or Zsh

# Installation
Bash:
1. Clone the repo: `git clone https://github.com/kimushun1101/ros2-aliases.git PATH_TO_CLONE`  
    For example:
    ```
    git clone https://github.com/kimushun1101/ros2-aliases.git ~/.ros2-aliases
    ```
2. Add ros2_aliases.bash to bashrc with your ROS_WORKSPACE: `echo 'source PATH_TO_CLONE/ros2_aliases.bash $ROS_WORKSPACE' >> ~/.bashrc`  
    For example:
    ```
    echo 'source ~/.ros2-aliases/ros2_aliases.bash ~/ros2_ws' >> ~/.bashrc
    ```
    (Optional) The 2nd argument sets colcon build command :
    Edit `~/.bashrc` as follows.
    ```
    source ~/.ros2-aliases/ros2_aliases.bash ~/ros2_ws "colcon build --symlink-install --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=0"
    ```

Zsh: Help wanted

# Usage

- `rahelp` shows `ros2_aliases help`. **(Important!) Just remember this.**
- `raload` loads `ros2_aliases config`.  
- `roscd` changes the working directory into the selected package directory under `$ROS_WORKSPACE/src`.  
- `chcbc` changes colcon build command with its arguments.
- `chrdi` changes ROS_DOMAIN_ID. If the argument is 0, ROS_LOCALHOST_ONLY=1 is set.

## Executable

| Command | Alias |
| --- | --- |
| `ros2 run` | `rrun` |
| `ros2 launch` | `rlaunch` |

## Topics

| Command | Alias |
| --- | --- |
| `ros2 topic list` | `rtlist` |
| `ros2 topic echo` | `rtecho`|
| `ros2 topic info` | `rtinfo`|

## Nodes

| Command | Alias |
| --- | --- |
| `ros2 node list` | `rnlist` |
| `ros2 node info` | `rninfo`|
| Killing a node | `rnkill`|

## Services

| Command | Alias |
| --- | --- |
| `ros2 service list` | `rslist` |

## Parameters

| Command | Alias |
| --- | --- |
| `ros2 param list` | `rplist` |
| `ros2 param get`  | `rpget`|
| `ros2 param set`  | `rpset`|

## Interface

| Command | Alias |
| --- | --- |
| `ros2 interface show`  | `rishow`|

## TF

| Command | Alias | Arguments |
| --- | --- | --- |
| `ros2 run tf2_tools view_frames` | `view_frames` | namespace of TF topic [Optional] |
| `ros2 run tf2_ros tf2_echo` | `tf2_echo`| source_frame [Required], target_frame [Required], namespace of TF topic [Optional] |

## Colcon

| Command | Alias |
| --- | --- |
| `cd $ROS_WORKSPACE` && `colcon build --symlink-install` | `cb` |
| `cd $ROS_WORKSPACE` && `colcon build --symlink-install --packages-select` | `cbp`|
| `cd $ROS_WORKSPACE` && `colcon build --symlink-install --cmake-clean-cache ` | `cbcc`|
| `cd $ROS_WORKSPACE` && `colcon build --symlink-install --cmake-clean-first ` | `cbcf`|
| `colcon list` | `cl` |

## Rosdep

| Command | Alias |
| --- | --- |
| `cd $ROS_WORKSPACE` && `rosdep install --from-paths src --ignore-src -y` | `rosdep_install` |

# Uninstall

Delete `source ~/.ros2-aliases/ros2_aliases.bash $ROS_WORKSPACE` in `~/.bashrc` then execute the following command.
```
rm -rf ~/.ros2-aliases
```

# Reference

- `ros2_utils.bash` : https://github.com/tonynajjar/ros2-aliases by Tony Najjar
- `yaml.sh` : https://github.com/jasperes/bash-yaml by Jonathan Peres
