/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2021, ROAS Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MI_ROS__MI_DRIVER_H_
#define MI_ROS__MI_DRIVER_H_

#include <string>
#include <vector>
#include <cmath>
#include <thread>

#include "ros/ros.h"
#include "serial/serial.h"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include "mi_ros/parser.h"
#include "mi_ros/Reset.h"

using namespace std;

class MiDriver : public Parser
{
public:
  MiDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~MiDriver();

  /**
   * \brief Initialize
   */
  bool init();

  /**
   * \brief Read message
   */
  void read();

  /**
   * \brief Reset IMU
   * \param req Request
   * \param resp Response
   */
  bool reset(mi_ros::Reset::Request& req, mi_ros::Reset::Response& resp);

  /**
   * \brief Publish IMU sensor data
   */
  void publishData();

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::ServiceServer srv_reset_;

  realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped> rp_rpy_;
  realtime_tools::RealtimePublisher<sensor_msgs::Imu> rp_imu_;

  geometry_msgs::Vector3Stamped rpy_;
  sensor_msgs::Imu imu_;

  /// For serial communication
  serial::Serial serial_;
  string port_;
  int32_t baud_;

  /// Publishing rate
  double rate_;

  /// Frame ID
  string frame_id_;
};

#endif  // MI_ROS__MI_DRIVER_H_