# archileon_ros_t1
ROS package for ArchiLeon

# セットアップの仕方（ソフトウェア）
- ROSのインストール
- 関連パッケージのインストール
    - Joy
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
`roslaunch archileon_ros_t1 display2.launch`
