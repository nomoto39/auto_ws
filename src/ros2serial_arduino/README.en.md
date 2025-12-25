[English](README.en.md) | [日本語](README.md)

# ROS2SERIAL_ARDUINO

## Description
Package to send and receive data using serial communication between ROS 2 node and Arduino Mega

- Receive /cmd_vel topic and send it to Arduino Mega via serial.
- Publish the serial data sent from Arduino Mega to the /serial_in topic of ROS 2.

## SETUP
#### 1. udev setup
```
# Copy the file
cp udev/99-usb-arduino-mega.rules /etc/udev/rules.d
# Reflect configuration
sudo udevadm control --reload-rules
```

#### 2. upload to Arduino
Write `ros2serial_arduino/Arduino/ros2serial_arduino/src/main.cpp`.

## USAGE
### serial communication

#### serial transmission
publish /cmd_vel, start joy or keyboard.
```
## joy
ros2 launch teleop_twist_joy teleop-launch.py
# key
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Run ROS 2 node.
```
ros2 run ros2serial_arduino serial_send_node
```

#### serial_receive
```
ros2 run ros2serial_arduino serial_receive_node
serial_receive_node
```