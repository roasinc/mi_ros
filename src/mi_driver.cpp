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

#include "mi_ros/mi_driver.h"

#define BUFFER_SIZE 26
#define RESET "$MIB,RESET*87"

MiDriver::MiDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : Parser(nh, nh_priv, rpy_, imu_)
  , nh_(nh)
  , nh_priv_(nh_priv)
  , port_("/dev/ttyUSB0")
  , baud_(38400)
  , rate_(50.0)
  , frame_id_("imu_link")
{
  nh_priv_.getParam("port", port_);
  nh_priv_.getParam("baud", baud_);
  nh_priv_.getParam("rate", rate_);
  nh_priv_.getParam("frame_id", frame_id_);
}

MiDriver::~MiDriver()
{
  serial_.close();
}

bool MiDriver::init()
{
  if (rate_ > 100.0)
  {
    ROS_WARN("MI driver does not support update rate of %f", rate_);
    return false;
  }

  srv_reset_ = nh_.advertiseService("reset", &MiDriver::reset, this);

  rp_rpy_.init(nh_, "rpy", 1);
  rp_rpy_.msg_.header.frame_id = frame_id_;

  rp_imu_.init(nh_, "data", 1);
  rp_imu_.msg_.header.frame_id = frame_id_;

  // Initial setting for serial communication
  serial_.setPort(port_);
  serial_.setBaudrate(baud_);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(to);

  try
  {
    serial_.open();
  }
  catch (serial::IOException& e)
  {
    // ROS_ERROR_STREAM_ONCE("Serial " << e.what());
  }

  // Check the serial port
  if (serial_.isOpen())
  {
    ROS_INFO("MI driver connected to %s at %i baud", port_.c_str(), baud_);
    return true;
  }
  else
  {
    ROS_ERROR("MI driver failed to connect to %s", port_.c_str());
    return false;
  }
}

void MiDriver::read()
{
  if (serial_.available())
  {
    uint8_t buffer[BUFFER_SIZE];
    serial_.read(buffer, BUFFER_SIZE);
    for (int8_t i = 0; i < BUFFER_SIZE; i++)
      parse(buffer[i]);

    // for (int8_t i = 0; i < BUFFER_SIZE; i++)
    //  ROS_INFO("Message[%d]: [0x%02x]", i, buffer[i]);
  }
}

bool MiDriver::reset(mi_ros::Reset::Request& req, mi_ros::Reset::Response& resp)
{
  if (req.reset = true)
  {
    stringstream msg;
    msg << RESET << "\r";
    serial_.write(msg.str());
  }

  return true;
}

void MiDriver::publishData()
{
  while (ros::ok())
  {
    if (rp_rpy_.trylock())
    {
      rp_rpy_.msg_.header.stamp = ros::Time::now();
      rp_rpy_.msg_.vector = rpy_.vector;
      rp_rpy_.unlockAndPublish();
    }

    if (rp_imu_.trylock())
    {
      rp_imu_.msg_.header.stamp = ros::Time::now();
      rp_imu_.msg_.orientation = imu_.orientation;
      rp_imu_.msg_.angular_velocity = imu_.angular_velocity;
      rp_imu_.msg_.linear_acceleration = imu_.linear_acceleration;
      rp_imu_.unlockAndPublish();
    }

    ros::Rate(rate_).sleep();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "mi_driver_node");
  ros::NodeHandle nh("imu");
  ros::NodeHandle nh_priv("~");

  auto mi = make_shared<MiDriver>(nh, nh_priv);

  if (mi->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create thread for publishing the IMU data
    thread publish([&mi]() -> void { mi->publishData(); });

    while (ros::ok())
      mi->read();

    spinner.stop();
  }

  return 0;
}