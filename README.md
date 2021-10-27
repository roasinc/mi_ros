MICROINFINITY XG6000 ROS Driver
===============================

Overview
--------

ROS driver for XG6000 IMU sensor.

ROS Interface
-----------

| Topic Name   | Type                             | Description             |
|--------------|----------------------------------|-------------------------|
| ``imu/data`` | ``sensor_msgs/Imu``              | IMU values              |
| ``imu/rpy``  | ``geometry_msgs/Vector3Stamped`` | Roll, Pitch, Yaw values |

| Service Name  | Type             | Description      |
|---------------|------------------|------------------|
| ``imu/reset`` | ``mi_ros/Reset`` | Reset the sensor |

Installation
------------

```
cd ~/catkin_ws/src/
git clone https://github.com/roasinc/mi_ros.git
cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```

Udev Setup
----------

```
roscd mi_ros/rulse/
sudo cp 41-mi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
