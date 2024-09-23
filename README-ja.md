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
    このスクリプトによる自動化の都合上、他に支障がなければ `VS Code` などの GUI エディターではなく、`vim` や `nano` など CLI エディターを設定してください。

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
    `editor` で [.env_example](/.env_example) をコピーした設定ファイル .env が開かれますので編集、保存して、閉じてください。
    `#` はコメントアウトです。
    `ROS_WORKSPACE` をご自身が現在開発中のワークスペースのパスに設定することを推奨します。
    ```
    ROS_WORKSPACE=${HOME}/ros2_ws
    ```

# 使い方

`rahelp` で `ros2_aliases` のヘルプを表示します。 **これだけ覚えていれば使えます。**  
これは **r**os2 **a**liases **help** を意味しています。
末尾には主要な環境変数の現在の値も表示します。確認に使用してください。
以下に機能を一覧で表示しますが、実際の使用イメージを掴みたい場合には、このページの最後に記載する[ハンズオンチュートリアル](#ハンズオンチュートリアル)をご覧ください。

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
    **set** **R**OS **W**ORK**S**PACE` を意味しています。
- `setrdi` で ROS_DOMAIN_ID をその引数の値に設定します。
    **set** **R**OS **D**omain **I**D` を意味しています。
    ```
    setrdi 40
    ```
    `setrdi 0` とすると ROS_LOCALHOST_ONLY=1 も設定されます。
- `setcbc` でビルドコマンドをその引数の文字列に設定します。
    **set** **c**olcon **b**uild **c**ommand` を意味しています。
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

# ハンズオンチュートリアル

Ubuntu 22.04 に ROS 2 Humble をインストール済みの PC で操作する例を示します。
後に出てくる `ROS_DISTRO` の部分を対応するバージョンに変更すれば異なる環境でも動くはずです。
このページの[準備](#準備)と[インストール](#インストール)が完了していることを前提とします。

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
```
`editor` が立ち上がるので、以下のように編集・保存して閉じてください。
```
# For ros2-aliases
ROS_DISTRO=humble # iron, jazzy, etc.
ROS_WORKSPACE=${HOME}/ros2-aliases_ws
# ROS_DOMAIN_ID=30
COLCON_BUILD_CMD="colcon build --symlink-install --parallel-workers $(nproc)"
# 以下略
```
具体的には `ROS_WORKSPACE` の先頭のコメントアウトを意味する `#` を外し、`ros2_ws` を `ros2-aliases_ws` に変更しましょう。
`ROS_WORKSPACE` が変更されたことは `echo` コマンドで確認できます。
```
echo $ROS_WORKSPACE
```
より簡単に確認する方法は ros2-aliases の `rahelp` コマンドを使うことです。
```
rahelp
```
ros2-aliases のコマンドリストとともに、現在の環境変数を確認できます。

以上で ros2-aliases にワークスペースを設定することができました。

## 環境変数の変更

特定のターミナルでは、環境変数をデフォルトから変更したいこともあるかもしれません。
そのために、`setenvfile` に引数として、同じフォーマットの異なるファイルを与えることもできます。
```
cp ~/.local/ros2-aliases/.env_example ~/ros2-aliases/.env
editor ~/ros2-aliases/.env
```
例えばここで `ROS_DOMAIN_ID` を 40 とします。
```
# For ros2-aliases
ROS_DISTRO=humble # iron, jazzy, etc.
ROS_WORKSPACE=${HOME}/ros2-aliases_ws
ROS_DOMAIN_ID=40
COLCON_BUILD_CMD="colcon build --symlink-install --parallel-workers $(nproc)"
# 以下略
```
`ROS_WORKSPACE` のときと同様に `ROS_DOMAIN_ID` の先頭のコメントアウトを意味する `#` を外し、`30` を `40` に変更しましょう。
保存して閉じたあと、`setenvfile` コマンドに渡してみましょう。
```
setenvfile ~/ros2-aliases/.env
```
引数として渡す場合には、`editor` は開かれません。
`rahelp` で `ROS_DOMAIN_ID` が変更されていることを確認しましょう。

`.env` ファイルを用意するまでもない場合には、`setrws`, `setrdi`, `setcbc` コマンドで個別に対応することも可能です。
```
setrws ~/ros2_ws # ワークスペースに src ディレクトリがない場合にはエラーになります。
```
```
setrdi 50
```
```
setcbc "colcon build --parallel-workers $(nproc)"
```

## パッケージの確認

`roscd` コマンドを使ってパッケージのディレクトリに移動してみましょう。
```
roscd
```
すると、fuzzy finder (以降 fzf と略します) で
```
  teleop_joy_component
> tb3_controller_cpp
```
というような選択画面に移ります。
方向キーで選択してもよいですし、少しの文字列をタイピングすることで fuzzy find も可能です。
選択するとそのディレクトリに移動します。
Esc で選択画面を抜けると `ROS_WORKSPACE` ディレクトリに移動します。

また、`roscd` コマンドにはパッケージ名を引数として渡すこともできます。
さらに、引数入力には Tab 補完が効くようになっています。
たとえば、`roscd tb3[tab]` で `roscd tb3_controller_cpp` まで補完され、Enter キーを押せば、`~/ros2-aliases_ws/src/tb3_controler_cpp` に移動します。

## ワークスペースのビルド

以下のデモでは、`tb3_controller_cpp` パッケージの使用をしていくのですが、このパッケージを使用するために必要な依存パッケージが存在します。
これらを `rosdep install` でインストールしますが、そのオプションまで含めて暗記している人はほとんどいないと思います。
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

毎回ワークスペースのすべてのパッケージをビルドしていると時間がかかり面倒です。
`colcon build` には `--packages-select` オプションがあり、指定したパッケージだけビルドする仕組みがあります。
しかし、毎回 `--packages-select` や該当パッケージを手打ちするのは大変です。
そこで ros2-aliases の `cbp` コマンドを使用しましょう。
```
cbp
```
`roscd` と同様に fzf による選択が要求されます。
ここでは `tb3_controller_cpp` を選択してみましょう。
すると、`tb3_controller_cpp` だけビルドされたことがわかります。

また、`cbp` コマンドもパッケージを引数として渡すことができ、引数入力には Tab 補完が効くようになっています。
さらに、引数として複数のパッケージを渡すことも可能です。
たとえば、`cbp tb3[tab]` で `cbp tb3_controller_cpp` まで補完され、スペースを挟んで続けて `teleop_joy_component` も補完入力して
```
cbp tb3_controller_cpp teleop_joy_component
```
と入力すれば、この 2 つのパッケージのみをビルドします。

ちなみに、`cb` でも `cbp` でも、`source ./install/setup.bash` まで自動で行うため、ビルドした ROS パッケージをそのまますぐに使用できます。
また、ファイルの変更によってビルドがうまく通らなくなってしまった場合に、`build`, `install`, `log` を削除して再ビルドされる方のために、`cbrm` や `cbprm` コマンドなども用意しております。

## 実行

それでは実際にノードを立ち上げてみましょう。
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
`tb3_controller_cpp` には `tb3_controller_node` しか含まれていませんが、パッケージ内にノードが多い場合には、このツールが有用であることを容易に想像できると思います。
`tb3_controller_node` を実行すると
```
ros2 run tb3_controller_cpp tb3_controller_node
[INFO] [1726983643.230067919] [tb3_controller]: tb3_controller node has been initialised
[INFO] [1726983643.230168383] [tb3_controller]: Kp : 1
[INFO] [1726983643.230187204] [tb3_controller]: T : 0.01
[INFO] [1726983643.230194561] [tb3_controller]: initial xd : 1
```
というような表示がされ、なにか動いていそうです。
ここまで確認できましたら、このノードは `Ctrl + C` で終了させましょう。

同じ要領で `ros2 launch` も試してみましょう。
ros2-aliases の `rlaunch` コマンドを実行してください。
```
rlaunch
```
ここで `tb3_controller_cpp` を選択すると、`simulation_and_controller.launch.yaml` と `turtlebot3_and_controller.launch.yaml` の２つの選択肢がでますが、今回は実機ではなくシミュレーターを起動することとし、`simulation_and_controller.launch.yaml` を選択します。
Gazebo で Turtlebot3 が制御されている様子を確認できるはずです。

別のターミナルで
```
ros2 topic pub /xd std_msgs/msg/Float32 "data: 3.0"
```
とコマンドすることで目標位置を変更して、Turtlebot3 を動かすことができます。
詳細についてはここでは割愛しますが、[tb3_controller の説明](https://github.com/kimushun1101/tb3_controller_cpp?tab=readme-ov-file#パラメーター調整)をご覧ください。

## デバッグ

`simulation_and_controller.launch.yaml` を立ち上げたまま以下の操作を行ってください。

現在出ているトピックを `rtlist` や `rtecho`、`rtinfo` などで確認しましょう。
これらは **r**os2 **t**opic **list**、**r**os2 **t**opic **echo**、**r**os2 **t**opic **info** のコマンドを実行してくれます。
同様に `rnlist`, `rninfo`, `rplist`, `rpget` なども試してみてください。
ROS 2 のコマンドを知っていれば、直感的に何が起こるか推測できるかと思います。

tf まわりのコマンドとしては `view_frames` があり、`frames_タイムスタンプ.pdf` が出力されるのでその中身を確認してみてください。
`tf_echo [source_frame] [target_frame] (namespace)` も使えます。
```
tf_echo base_footprint base_scan
```
とすれば、`base_footprint` から見た `base_scan` の位置・姿勢を取得できます。

## 後片付け

ここまで一連の開発の流れを通して、ros2-aliases の基本的な使い方を習得できたかと思います。
ハンズオンチュートリアルを終了とし、後片付けを行いましょう。
まずは `setenvfile` コマンドで設定を元に戻します。
とくに `ROS_WORKSPACE` は、ご自身が現在開発中のワークスペースに設定するとよいでしょう。
あとは今回のお試しのために作成した `ros2-aliases_ws` を削除したら完了です。
```
rm -rf ~/ros2-aliases_ws
```
以上、お疲れ様でした。
