[日本語](/README-ja.md) | [English](/README.md)

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

- Bash

- editor  
    (Optional) Configuration of `editor` is recommended.
    ```
    sudo update-alternatives --config editor
    ```

# Installation
1. Clone this repository:
    ```
    git clone https://github.com/kimushun1101/ros2-aliases.git $HOME/.local/ros2-aliases
    ```
2. Add ros2_aliases.bash to bashrc:
    ```
    echo 'source $HOME/.local/ros2-aliases/ros2_aliases.bash' >> ~/.bashrc
    ```
3. Apply the bashrc update and edit .env for initial settings:
    ```
    source ~/.bashrc
    setenvfile
    ```
    `#` means comment out. 
    Recommended is to set `ROS_WORKSPACE` to the path of the workspace you use most often.
    ```
    ROS_WORKSPACE=${HOME}/ros2_ws
    ```

# Usage

`rahelp` shows `ros2_aliases help`. **(Important!) Just remember this.**  
The current key environment variables can also be viewed.

## Environment variables

- `setenvfile` sets environment variables with a file. This function can also take an env file as an argument.
    | Environment variable | Description |
    | --- | --- |
    | `ROS_DISTRO` | humble, Iron, Jazzy, etc. |
    | `ROS_WORKSPACE` | full path to your ROS 2 workspace |
    | `ROS_DOMAIN_ID` | refer to [the official documentation](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html) |
    | `COLCON_BUILD_CMD` | build command with options |
- `setrws` sets ROS 2 workspace.
- `setrdi` sets ROS_DOMAIN_ID. If the argument is 0, ROS_LOCALHOST_ONLY=1 is set.
- `setcbc` sets colcon build command with its arguments.

## Roscd

`roscd` sets the working directory into the selected package directory under `$ROS_WORKSPACE/src`.

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
| `ros2 topic bw` | `rtbw`|

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
| `colcon build --symlink-install` | `cb` |
| `colcon build --symlink-install --packages-select` | `cbp`|
| `colcon test --packages-select` and `colcon test-result --verbose` | `ctp`|
| `colcon build --symlink-install --cmake-clean-cache ` | `cbcc`|
| `colcon build --symlink-install --cmake-clean-first ` | `cbcf`|
| `rm build install log` and `colcon build --symlink-install` | `cbrm`|
| `rm build install log` for selected packages and `colcon build --symlink-install --packages-select` | `cbprm`|
| `colcon list` | `cl` |

## Rosdep

| Command | Alias |
| --- | --- |
| `cd $ROS_WORKSPACE` && `rosdep install --from-paths src --ignore-src -y` | `rosdep_install` |

# Uninstall

Delete `source $HOME/.local/ros2-aliases/ros2_aliases.bash` in `~/.bashrc` then remove this directory.
```
sed -i '\|source $HOME/.local/ros2-aliases/ros2_aliases.bash|d' ~/.bashrc
rm -rf $HOME/.local/ros2-aliases 
```

# Reference

- `ros2_utils.bash` : https://github.com/tonynajjar/ros2-aliases by Tony Najjar
