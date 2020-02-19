# tank_motor

## Notes
> ### Creating a catkin Package
>> 1. cd ~/catkin_ws/src
>> 2. catkin_create_pkg tank_motor roscpp rospy std_msgs message_generation


> ### Building a catkin workspace and sourcing the setup file
>> 1. cd ~/catkin_ws
>> 2. catkin_make
>> 3. . ~/catkin_ws/devel/setup.bash

## Clone and build the package
> ### tank_camera
>> 1. cd ~/catkin_ws/src
>> 2. git clone https://github.com/KhairulIzwan/tank_motor.git
>> 3. cd ~/catkin_ws
>> 4. catkin_make
>> 5. . ~/catkin_ws/devel/setup.bash

## Required Package
> ### rosserial
>> 1. cd ~/catkin_ws/src
>> 2. git clone https://github.com/ros-drivers/rosserial.git
>> 3. cd ~/catkin_ws
>> 4. catkin_make
>> 5. . ~/catkin_ws/devel/setup.bash
>>
>>> #### Install ros_lib into the Arduino Environment
>>>> 1. roscd tank_motor/installArduinoIDE/
>>>> 2. ./installArduinoIDE.sh

# Usage

## Tele-operation
> ### Using Arduino
>> Arduino IDE
>>> ROS-node-dc-motor-control.ino

>> Terminal # 1
>>> roscore

>> Terminal # 2
>>> rosrun rosserial_arduino serial_node.py /dev/ttyACM0 <-- refer to your arduino port

>> Terminal # 3
>>> rosrun tank_motor teleop_key.py

> ### Using Raspberry Pi
>> Terminal # 1
>>> roscore

>> Terminal # 2
>>> rosrun tank_motor motor_control.py

>> Terminal # 3
>>> rosrun tank_motor teleop_key.py
