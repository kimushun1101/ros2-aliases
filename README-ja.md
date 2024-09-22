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
    環境変数の説明は以下の表のとおりです。
    | Environment variable | Description |
    | --- | --- |
    | `ROS_DISTRO` | humble, Iron, Jazzy など |
    | `ROS_WORKSPACE` | 使用するワークスペースのフルパス |
    | `ROS_DOMAIN_ID` | [公式ページ](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html) を参照 |
    | `COLCON_BUILD_CMD` | ビルドのコマンド |
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
`ROS_DOMAIN_ID` が変更されたことは `echo` コマンドで確認できます。
```
echo $ROS_DOMAIN_ID
```
より簡単に確認する方法は `rahelp` コマンドを使うことです。
```
rahelp
```
ros2-aliases のコマンドリストとともに、現在の環境変数の確認もできます。
以上で ros2-aliases にワークスペースを教えることは完了です。

## 環境変数の変更

特定のターミナルでは、環境変数をデフォルトから変更したいこともあるかもしれません。
そのために、`setenvfile` に引数として、同じフォーマットの異なるファイルを与えることもできます。
```
cp ~/.local/ros2-aliases/.env_example ~/ros2-aliases/.env
editor ~/ros2-aliases/.env
```
例えばここで `ROS_DOMAIN_ID` を 40 としましょう。
```
# For ros2-aliases
ROS_DISTRO=humble # jazzy
ROS_WORKSPACE=${HOME}/ros2-aliases_ws
COLCON_BUILD_CMD="colcon build --symlink-install --parallel-workers $(nproc)"
ROS_DOMAIN_ID=40
# 以下略
```
保存して閉じたあと、`setenvfile` コマンドに渡してみましょう。
```
setenvfile ~/ros2-aliases/.env
```
引数として渡す場合には、`editor` は開かれません。
`rahelp` で `ROS_DOMAIN_ID` が変更されていることを確認しましょう。

`.env` ファイルを用意するまでもない場合には、`setrws`, `setrdi`, `setcbc` コマンドで対応してください。
```
setrws ~/ros2_ws # ワークスペースに src ディレクトリがない場合にはエラーになります。
setrdi 50
setcbc "colcon build --parallel-workers $(nproc)"
```

## パッケージの確認

`roscd` コマンドを使ってパッケージのディレクトリに移動してみましょう。
```
roscd
```
すると
```
  teleop_joy_component
> tb3_controller_cpp
```
というような選択画面に移ります。
方向キーで選択してもよいですし、キーボードで少しタイピングすることで fuzzy find (以降 fzf と略します)もしてくれます。
選択するとそのディレクトリに移動します。
選択を Esc で抜けると `ROS_WORKSPACE` ディレクトリに移動します。

## ワークスペースのビルド

以下のデモでは、`tb3_controller_cpp` パッケージの使用をしていくのですが、このパッケージを使用するために必要な依存パッケージが存在します。
これらを `rosdep install` でインストールしますが、オプションまで含めて暗記している人はほとんどいないと思います。
そこで、ros2-aliases の `rosdep_install` コマンドを使用しましょう。
ちゃんと整備されているパッケージであれば、依存関係を解決してくれます。
`rosdep` で依存関係を解決できないパッケージに対しては、`roscd` コマンドでパッケージのディレクトリに移動し、それぞれの README.md などを確認して環境構築を行ってください。

準備ができたところで、ワークスペースをビルドしましょう。
ros2-aliases の `cb` コマンドを使えば、カレントディレクトリがどこであっても `ros2-aliases_ws` がビルドできます。
通常、`~/ros2-aliases_ws` にて `colcon build` をしなければなりません。
`cb` コマンドではこれを自動で行ってくれます。
```
cb
```

## 単体パッケージのビルド

毎回ワークスペースのすべてのパッケージをビルドしていると時間がかかり面倒です。
`colcon build` には `--packages-select` オプションがあり、指定したパッケージだけビルドする仕組みがあります。
しかし、毎回 `--packages-select` や該当パッケージを手打ちするのは大変です。
そこで ros2-aliases の `cbp` コマンドを使用しましょう。
```
cbp
```
`roscd` と同様に fzf による選択が要求されますが、ここでは `tb3_controller_cpp` を選択してみましょう。
すると、`tb3_controller_cpp` だけビルドされたことがわかります。

ちなみに、`cb` でも `cbp` でも、`source ./install/setup.bash` まで自動で行うため、すぐに ROS パッケージが使用できます。

## 実行

それでは、実際にノードを立ち上げてみましょう。
たとえば `ros2 run tb3_controller_cpp tb3_controller_node` を立ち上げたいとします。
長いですね。またパッケージ名やノード名をうろ覚えですと面倒臭さが倍増します。
ros2-aliases では `rrun` コマンドが使用できます。
```
rrun
```
まずは、パッケージ名を fzf します。
ここでわかるように独自パッケージだけでなく、apt install した ROS パッケージの検索も可能です。
ここでは `tb3_controller_cpp` を選択してみましょう。
つぎに、`tb3_controller_cpp` に含まれるノード名を fzf してくれます。
`tb3_controller_cpp` には `tb3_controller_node` しか含まれていませんが、ノードが多いパッケージで有用であることが容易に想像できると思います。
`tb3_controller_node` を実行すると
```
ros2 run tb3_controller_cpp tb3_controller_node
[INFO] [1726983643.230067919] [tb3_controller]: tb3_controller node has been initialised
[INFO] [1726983643.230168383] [tb3_controller]: Kp : 1
[INFO] [1726983643.230187204] [tb3_controller]: T : 0.01
[INFO] [1726983643.230194561] [tb3_controller]: initial xd : 1
```
というような表示がされ、なにか動いていそうです。
`Ctrl + C` で終了させましょう。

同様の要領で `ros2 launch` も試してみましょう。
ros2-aliases の `rlaunch` コマンドを実行してみましょう。
```
rlaunch
```
ここで `tb3_controller_cpp` を選択すると、`simulation_and_controller.launch.yaml` と `turtlebot3_and_controller.launch.yaml` の２つの選択肢がでますが、今回は実機ではなくシミュレーターを起動することとし、`simulation_and_controller.launch.yaml` を選択します。
Turtlebot3 が P 制御されている様子が Gazebo で確認できるはずです。

## デバッグ

`simulation_and_controller.launch.yaml` を立ち上げたまま以下の操作を行ってください。
現在出ているトピックを `rtlist` や `rtecho`、`rtinfo` などで確認しましょう。
同様に `rnlist`, `rninfo`, `rplist`, `rpget` なども試してみてください。
fzf による検索になれてくると、直感的に何が起こっているか推測できると思います。

tf まわりのコマンドとしては `view_frames` があり、`frames_タイムスタンプ.pdf` が出力されるのでその中身を確認してみてください。
`tf_echo [source_frame] [target_frame] (namespace)` も使えます。
```
ros2 run tf2_ros tf2_echo base_footprint base_scan
```
とすれば、`base_footprint` から見た `base_scan` の位置・姿勢を取得できます。

## 後片付け

`setenvfile` コマンドで設定をもとに戻しましょう。
とくに `ROS_WORKSPACE` は、あなたが今開発を進めているワークスペースにセットするとよいでしょう。
あとは今回のお試しのために作成した `ros2-aliases_ws` を削除したら完了です。
```
rm -rf ~/ros2-aliases_ws
```
以上、お疲れ様でした。
