# seto_scararobot2 について

## 準備

### インストール

事前に以下のパッケージをインストールする。

```
$ sudo apt-get update
$ sudo apt install ros-noetic-joint-state-publisher-gui
$ sudo apt install ros-noetic-joint-trajectory-controller
$ sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
$ sudo apt-get install ros-noetic-dynamixel-workbench-controllers
$ pip install flask
$ pip install serial
```

### ビルド

1. ROSワークスペースのsrcフォルダの中にクローンして、catkin_make

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/dreamdrive/seto_scararobot2.git
$ cd ..
$ catkin_make
```

## 使い方

### 使い方1 (joint_state_publisherでrviz上のスカラロボットをスライダを動かせます。)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2_description display.launch
```

### 使い方2 (rviz上のスカラロボットが勝手に動きます。)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2 move_test1_rviz.launch
```
joint_state_publisherの代わりに、move_testノードが立ち上がります。

### 使い方3 (IKで座標を指定してrviz上のスカラロボットを動かす。)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2 move_test2_rviz.launch
```
joint_state_publisher_guiが立ち上がりますので、スライダを動かすことでスカラロボットが動きます。単位は(mm)です。

### 使い方4 (ロボットアームをwebアプリから動かす。【本命】) ※実機が必要 

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2 move_arm_hardware.launch
```

2. webブラウザで[localhost:5000](localhost:5000)にアクセスする

※ 1.のコマンドの代わりに、seto_scararobot2.shを実行してもOK、その場合レイテンシタイマーの設定もシェルの中で行います。

### 使い方5 (gazenoのロボットアームをwebアプリから動かす。

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2 move_arm_gazebo.launch
```
2. webブラウザで[localhost:5000](localhost:5000)にアクセスする

### FTDIドライバのレイテンシタイマーの変更

```
 $ sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
 $ echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## ヒント

### catkin_makeを実行してもroslaunchでlaunchファイルが実行できない or 予測変換で出現しない
以下のコマンドを実行する。
```
$ source devel/setup.bash
```
それでも直らない場合はこちらを実行する。
```
$ rospack profile
```

## 予定・課題
* (n/a)

## 履歴
* 2021/07/12 :  とりあえず、１号機から移植
* 2021/07/27 :  noeticに対応。move_test1とmove_test2を2号機モデルに対応。
* 2021/08/16 :  ver.0.9リリース

## Author

* 瀬戸内ROS勉強会
  * https://ros.xrea.jp/

## License

This is under [Apache License 2.0](./LICENSE).
