[日本語](/README-ja.md) | [English](/README.md)

# ros2　aliases

ROS 2 を開発するときに便利なエイリアスと関数

![](https://github.com/tonynajjar/ros2-aliases/blob/main/usage.gif)

# 準備

- [fzf](https://github.com/junegunn/fzf#installation)
  Ubuntu の場合は、apt でインストールできます。 
  ```
  sudo apt install fzf
  ```
  For more install options refer to the documentation

- Bash (Zsh はまだできていません)

# インストール

1. このリポジトリをクローンしてください。`git clone https://github.com/kimushun1101/ros2-aliases.git PATH_TO_CLONE`  
    ホームディレクトリに隠しフォルダとして入れる場合には以下の通り
    ```
    git clone https://github.com/kimushun1101/ros2-aliases.git ~/.ros2-aliases
    ```
2. bashrc にワークスペースのディレクトリを読み込むように追加してください。`echo 'source PATH_TO_CLONE/ros2_aliases.bash $ROS_WORKSPACE' >> ~/.bashrc`  
    隠しフォルダにインストールした状態で、`~/ros2_ws` を指定する場合には以下のコマンドを実行
    ```
    echo 'source ~/.ros2-aliases/ros2_aliases.bash ~/ros2_ws' >> ~/.bashrc
    ```

# 使い方

- `rahelp` で `ros2_aliases` のヘルプを表示します。 **これだけ覚えていれば使えます**
  `ros2 aliases help` を意味しています。
- `raload` で `ros2_aliases`の設定をロードします。
  `ros2 aliases load` を意味しています。
- `roscd` で `$ROS_WORKSPACE/src` 以下にあるパッケージのディレクトリに移動します。  
- `chcbc` でその引数をデフォルトのビルドコマンドに変更します。
  `change colcon build command` を意味しています。
- `chrdi` でその引数に ROS_DOMAIN_ID を変更します。`chrdi 0` とすると ROS_LOCALHOST_ONLY=1 が設定されます。
  `chage ROS Domain ID` を意味しています。

## 実行

| Command | Alias |
| --- | --- |
| `ros2 run` | `rrun` |
| `ros2 launch` | `rlaunch` |

## トピック

| Command | Alias |
| --- | --- |
| `ros2 topic list` | `rtlist` |
| `ros2 topic echo` | `rtecho`|
| `ros2 topic info` | `rtinfo`|

## ノード

| Command | Alias |
| --- | --- |
| `ros2 node list` | `rnlist` |
| `ros2 node info` | `rninfo`|
| Killing a node | `rnkill`|

## サービス

| Command | Alias |
| --- | --- |
| `ros2 service list` | `rslist` |

## パラメーター

| Command | Alias |
| --- | --- |
| `ros2 param list` | `rplist` |
| `ros2 param get`  | `rpget`|
| `ros2 param set`  | `rpset`|

## インターフェース

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
| `rm build install log` for selected packages and `colcon build --symlink-install --packages-select` | `cbrm`|
| `colcon list` | `cl` |

## rosdep

| Command | Alias |
| --- | --- |
| `cd $ROS_WORKSPACE` && `rosdep install --from-paths src --ignore-src -y` | `rosdep_install` |

# アンインストール

`~/.bashrc` から `source PATH_TO_CLONE/ros2_aliases.bash $ROS_WORKSPACE` の行を削除してください。
その後クローンしたディレクトリを削除してください。
```
rm -rf ~/.ros2-aliases # PATH_TO_CLONE
```

# 参考

- `ros2_utils.bash` : https://github.com/tonynajjar/ros2-aliases by Tony Najjar
- `yaml.sh` : https://github.com/jasperes/bash-yaml by Jonathan Peres
