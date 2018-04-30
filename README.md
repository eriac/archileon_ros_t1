# archileon_ros_t1
ROS package for ArchiLeon

# セットアップの仕方（ソフトウェア）
- ROSのインストール
- 関連パッケージのインストール
    - Joy  
    `sudo apt-get install ros-kinetic-joy`
    - jsk-visualization  
    `sudo apt-get install ros-kinetic-jsk-rviz-plugins`  
    `sudo apt-get install ros-kinetic-jsk-visualization`  
    `sudo apt-get upgrade`
- リポジトリのclone  
`cd ~/catkin_ws/src`  
`git clone git@github.com:eriac/archileon_ros_t1.git`  
- ビルド  
`cd ~/catkin_ws`  
`catkin_make`
- シリアル通信の権限を取得  
`sudo gpasswd -a [USERNAME] dialout`  
`sudo reboot`

# セットアップの仕方（ハードウェア）
- Arduinoに4つのタイヤセットをつなぐ  
FL(左前)がch.0、FR:1、BL:ch.2、BR:ch.3  
- Arduinoにシリアル通信ケーブルを付ける
- 電源を繋げる
- PCにジョイスティックを刺す

# ROSの起動
`roslaunch archileon_ros_t1 robot.launch`

# memo
## 実機とRvizのロボットの曲がり方が違うかも
move_driver.cpp中のline 11  
float curve_factor=0.6;//really 1?  
でロボットの曲がり方の微調整をしている。小さいほうが強く曲がる  
実機を動かすときは調整して
