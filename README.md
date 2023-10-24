# This repository will not be maintained further.

# mi_ros
[![Licence](https://img.shields.io/badge/License-BSD--3-green.svg)](https://opensource.org/license/bsd-3-clause/)
[![ubuntu20](https://img.shields.io/badge/-UBUNTU_20.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/focal/)
[![noetic](https://img.shields.io/badge/-NOETIC-blue?style=flat-square&logo=ros)](https://wiki.ros.org/noetic)

## Overview
ROS driver package for XG6000 IMU sensor

## Specification
|                   Parameter                  | Value | Unit    |
|:--------------------------------------------:|-------|---------|
|          Measurement - Angular rate          | ±250  | deg/sec |
| Measurement - Acceleration                   | ±2    | g       |
| Bandwidth - Angular rate                     | 12    | Hz      |
| Bandwidth - Acceleration                     | 62.5  | Hz      |
| Yaw axis angular rate - Scale factor error   | 0.3   | %       |
| Yaw axis angular rate - Bias drift           | 10    | deg/hr  |
| Yaw axis relative angle - Proportional error | 0.3   | %       |
| Yaw axis relative angle - Drift error        | 10    | deg/hr  |
| Roll, pitch accuracy - Static error          | 0.3   | deg     |
| Roll, pitch accuracy - Dynamic error         | 0.7   | deg     |
| Resolution - Angular rate                    | 0.01  | deg/sec |
| Resolution - Angle                           | 0.01  | deg     |
| Resolution - Acceleration                    | 1     | mg      |
| Data rate                                    | 100   | Hz      |

## ROS Interface

| Topic Name   | Type                             | Description             |
|--------------|----------------------------------|-------------------------|
| ``imu/data`` | ``sensor_msgs/Imu``              | IMU data              |
| ``imu/rpy``  | ``geometry_msgs/Vector3Stamped`` | roll, pitch, yaw data |

| Service Name  | Type                 | Description      |
|---------------|----------------------|------------------|
| ``imu/reset`` | ``std_srvs/Trigger`` | Reset the sensor |

## Installation
```
cd ~/catkin_ws/src/
git clone https://github.com/roasinc/mi_ros.git

cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```

## Usage
```
sudo chmod 666 /dev/ttyUSB0
roslaunch mi_ros mi.launch
```
