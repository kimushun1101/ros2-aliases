[日本語](/README-ja.md) | [English](/README.md)

# ros2-aliases

ROS 2 を開発するときに便利なエイリアス(関数)を提供します。

![](https://github.com/tonynajjar/ros2-aliases/blob/main/usage.gif)

# 準備

- Bash
- [fzf](https://github.com/junegunn/fzf#installation)  
    Ubuntu の場合は、apt でインストールできます。 
    ```
    sudo apt install fzf
    ```
- editor  
    (任意) `editor` コマンドで開くエディターを指定しておくことをオススメします。
    ```
    sudo update-alternatives --config editor
    ```
    VS Code なども選べますが `vim` や `nano` など CLI エディターを設定することをオススメします。

# インストール

1. このリポジトリをクローンしてください。  
    以降、例として `$HOME/.local` にクローンする場合を記載します。
    ```
    git clone https://github.com/kimushun1101/ros2-aliases.git $HOME/.local/ros2-aliases
    ```
2. bashrc に `ros2_aliases.bash` を読み込むように追加してください。  
    ```
    echo 'source $HOME/.local/ros2-aliases/ros2_aliases.bash' >> ~/.bashrc
    ```
3. .bashrc の変更を反映させた後、環境変数ファイルを設定する。
    ```
    source ~/.bashrc
    setenvfile
    ```
    `editor` で [.env_example](/.env_example) をコピーした設定ファイル `.env` が開かれますので編集、保存して、閉じてください。
    `#` はコメントアウトです。
    `ROS_WORKSPACE` をご自身がよく使うワークスペースのパスに設定することを推奨します。
    ```
    ROS_WORKSPACE=${HOME}/ros2_ws
    ```

# 使い方

`rahelp` で `ros2_aliases` のヘルプを表示します。 **これだけ覚えていれば使えます。**  
`ros2 aliases help` を意味しています。
末尾には主要な環境変数の現在の値も表示します。確認に使用してください。

## 環境変数

- `setenvfile` でデフォルトの環境変数を設定します。
    `set environment` を意味しています。
    引数なしで実行した場合には、[インストールの3](#インストール)で説明した動作をします。
    環境変数ファイルを引数として渡すことで、たとえばワークスペース独自の環境変数を設定することもできます。
    ```
    # ~/ros2_ws に .env ファイルを作成しておく
    setenvfile ~/ros2_ws/.env
    ```
- `setrws` で ROS 2 workspace を設定します。
    `set ROS WORKSPACE` を意味しています。
- `setrdi` で ROS_DOMAIN_ID をその引数の値に設定します。
    `set ROS Domain ID` を意味しています。
    ```
    setrdi 40
    ```
    `setrdi 0` とすると ROS_LOCALHOST_ONLY=1 も設定されます。
- `setcbc` でビルドコマンドをその引数の文字列に設定します。
    `set colcon build command` を意味しています。
    ```
    setcbc "colcon build --symlink-install --parallel-workers $(nproc)"
    ```

## roscd

`roscd` で `$ROS_WORKSPACE/src` 以下にあるパッケージのディレクトリに移動します。  
引数なしだと `fzf` による検索に入り、引数としてパッケージ名を渡せばそのパッケージのディレクトリに移ります。Tab 補完も有効です。
```
roscd pkg_name
```

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
| `ros2 topic bw` | `rtbw`|

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
| `rm build install log` for selected packages and `colcon build --symlink-install --packages-select` | `cbprm`|
| `colcon list` | `cl` |

## rosdep

| Command | Alias |
| --- | --- |
| `cd $ROS_WORKSPACE` && `rosdep install --from-paths src --ignore-src -y` | `rosdep_install` |

# アンインストール

`~/.bashrc` から `source $HOME/.local/ros2-aliases/ros2_aliases.bash` の行を削除してください。
その後クローンしたディレクトリを削除してください。
```
sed -i '\|source $HOME/.local/ros2-aliases/ros2_aliases.bash|d' ~/.bashrc
rm -rf $HOME/.local/ros2-aliases 
```

# 引用

- `ros2_utils.bash` : https://github.com/tonynajjar/ros2-aliases by Tony Najjar


---

# おまけ（簡単なチュートリアル）

Ubuntu 22.04 に ROS 2 Humble をインストール済みの PC で操作する例を示します。

## ワークスペースの設定

ホームディレクトリに `ros2-aliases_ws` を作成して、`src` ディレクトリにいくつかパッケージを追加していきましょう。
```
mkdir -p ~/ros2-aliases_ws/src
cd ~/ros2-aliases_ws/src
git clone https://github.com/kimushun1101/tb3_controller_cpp.git
git clone https://github.com/kimushun1101/teleop_joy_component_template.git
```

つぎに ros2-aliases の `setenvfile` コマンドで `.env` ファイルを以下のように編集しましょう。
```
setenvfile
# editor が立ち上がるので、以下のように編集・保存して閉じる。
# For ros2-aliases
ROS_DISTRO=humble # jazzy
ROS_WORKSPACE=${HOME}/ros2-aliases_ws
COLCON_BUILD_CMD="colcon build --symlink-install --parallel-workers $(nproc)"
# ROS_DOMAIN_ID=30
# 以下略
```
以上でワークスペースの設定は完了です。

## ワークスペースのビルド
`cb` コマンドでカレントディレクトリがどこであっても `ros2-aliases_ws` がビルドできることを確認しましょう。
たとえば、ホームディレクトリから `cb` すると以下の通りです。
```
cd
pwd
cb
```
ダウンロードディレクトリであっても同じ結果が得られます。
```
cd ~/Downloads
pwd
cb
```

## 単体パッケージのビルド

## 実行

## デバッグ


## 後片付け

## おわりに

