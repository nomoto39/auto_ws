[日本語](README.md) | [English](README.en.md) | 

# ROS2SERIAL_ARDUINO
## Description
ROS 2ノードとArduino Megaの間でシリアル通信を使ってデータを送受信するためのパッケージ．

- /cmd_velトピックを受け取り，serialでArduino Megaに送る．
- Arduino Mega から送られたserialデータをROS 2の/serial_inトピックにpublishする．

## SETUP
#### 1. udevの設定
```
# ファイルをコピー
cp udev/99-usb-arduino-mega.rules /etc/udev/rules.d
# 設定の反映
sudo udevadm control --reload-rules
```

#### 2. Arduinoへアップロード
`ros2serial_arduino/Arduino/ros2serial_arduino/src/main.cpp`を書き込む．

## USAGE
### serial通信

#### serial送信
/cmd_vel をpublishする．joyまたは，keyboardを起動する．
```
# joy
ros2 launch teleop_twist_joy teleop-launch.py
# key
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
ROS 2ノードの実行
```
ros2 run ros2serial_arduino serial_send_node
```

#### serial受信
```
ros2 run ros2serial_arduino serial_receive_node
```