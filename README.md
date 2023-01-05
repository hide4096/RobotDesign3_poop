# ロボット設計製作論3 知能コース poop班

CRANE-X7でハノイの塔を解くプログラムです。ただし、ハノイの塔はうんこの形をしています。

本パッケージはオリジナルである[rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros)をもとに、未来ロボティクス学科で開講された講義内でpoop班が製作したものです。

## About

CRANE-X7で２段のハノイの塔を解きます。ハノイの塔のモデルは[Pile of Poo Emoji (U+1F4A9)](https://www.thingiverse.com/thing:1682665)に取っ手と凹凸を追加したものを使用しています。

### 動作の流れ
うんこを模したハノイの塔には、各段にARマーカーを貼りつけています。
また、塔の置き場所を指定するために地面にもARマーカーが貼られています。

`rosrun RobotDesign3_poop main.py`を実行すると、以下の流れで塔を移動します
1. 塔の移動パターンを生成する
1. アーム先端のカメラでARマーカーを検索
1. 生成したパターンから、動かす塔の番号と置き場所の番号を取得する
1. 塔の持ち手をつかむ
1. 塔を垂直に持ち上げて、置き場所まで移動する
1. 垂直に下して、持ち手を離す
1. 塔の移動が終わるまで2~6を繰り返す

[動作の様子](https://twitter.com/tukugami_cola/status/1603314567282454528)

## Requirement
- ROS Noetic
- [crane_x7_ros](https://github.com/rt-net/crane_x7_ros)
- [crane_x7_d435](https://github.com/Kuwamai/crane_x7_d435)
- [ar_track_alvar](https://github.com/ros-perception/ar_track_alvar)
- [realsens-ros](https://github.com/IntelRealSense/realsense-ros)

次の環境で動作を確認しています
- Ubuntu 20.04
- ROS Noetic
- Rviz 1.13.19
- Gazebo 11.11.0

## Installation

- Requirementにあるパッケージをインストールします
    - [crane_x7_ros](https://github.com/rt-net/crane_x7_ros)
    - [crane_x7_d435](https://github.com/Kuwamai/crane_x7_d435)
    - 実機で動作させる場合は[realsens-ros](https://github.com/IntelRealSense/realsense-ros)も
    - [ar_track_alvar](https://github.com/ros-perception/ar_track_alvar)
        - Noeticにインストールする場合はブランチを指定してあげる必要があります
        - `git clone https://github.com/machinekoder/ar_track_alvar.git -b noetic-devel`
        - [参考](https://github.com/ros-perception/ar_track_alvar/issues/82)

- このリポジトリをクローンして、ビルドします
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/hide4096/RobotDesign3_poop
    cd ~/catkin_ws && catkin_make
    source ~/catkin_ws/devel/setup.bash
    ```

## Usage

- シミュレーターを使う場合
    ```
    roslaunch roslaunch RobotDesign3_poop poop_sim.launch
    rosrun RobotDesign3_poop main.py
    ```
- 実機を使う場合
    - 準備
        - main.pyのmarker_poopで指定された番号のARマーカーをうんこに貼ります
        - 同様に、marker_groundで指定された番号のARマーカーをアームが届く範囲に貼ります
        - 使用するマーカーのサイズは一辺が27mmです。
    - 実行
        次のコマンドを入力すると動作を開始します
        ```
        roslaunch crane_x7_d435 bringup.launch
        roslaunch realsense2_camera rs_camera.launch
        roslaunch RobotDesign3_poop ar_realsense.launch
        rosrun RobotDesign3_poop main.py
        ```

## Slides

- [中間報告(10/24)](https://hide4096.github.io/RobotDesign3_poop/slides/1024/#/)
- 中間発表(11/14?)
- [最終発表(12/19)](https://hide4096.github.io/RobotDesign3_poop/slides/1219/#/)

## License

- CRANE-X7の操作にはアールティ社様の[crane_x7_ros](https://github.com/rt-net/crane_x7_ros)を使用しています。
- CRANE-X7のD435付きモデルは[crane_x7_d435](https://github.com/Kuwamai/crane_x7_d435)を使用しています。
- ARマーカーの検出には[ar_track_alvar](https://github.com/ros-perception/ar_track_alvar)を使用しています。
- Realsenseのドライバとして、[realsens-ros](https://github.com/IntelRealSense/realsense-ros)を使用しています。
- うんこのモデルにはAlexander Thomas　DrLex様の[Pile of Poo Emoji (U+1F4A9)](https://www.thingiverse.com/thing:1682665)を使用しています。またPile of Poo Emoji (U+1F4A9)にはCRANE-X7のハンドで持ちやすいように取手を追加、ハノイの塔になるように３分割等の改変を行いました。
- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます．

© 2022 Aso Hidetoshi

© 2022 Shusei Aida
