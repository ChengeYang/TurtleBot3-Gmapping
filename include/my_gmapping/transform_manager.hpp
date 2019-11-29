/****************************************************************************
 *
 * Copyright (c) 2018-2019 Chenge Yang. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Chenge Yang nor the names of their contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Handle everything related to tf and transform in ROS.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef TRANSFORM_MANAGER_HPP
#define TRANSFORM_MANAGER_HPP

#include <iostream>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

#include "my_gmapping/utils.hpp"

namespace my_gmapping
{
class TransformManager
{
private:
  // TF subscriber and publisher
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Frame names
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

public:
  // Construtor
  TransformManager() : tf_listener_(tf_buffer_)
  {
  }

  TransformManager(std::string map_frame, std::string odom_frame,
                   std::string base_frame)
    : tf_listener_(tf_buffer_)
    , map_frame_(map_frame)
    , odom_frame_(odom_frame)
    , base_frame_(base_frame)
  {
  }

public:
  // Read odometry data from /tf and transfer to Pose data
  // void getOdometryData(Pose& pose)
  // {
  //   try
  //   {
  //     geometry_msgs::TransformStamped transform_stamped =
  //         tf_buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
  //     pose.timestamp_ = 0;
  //     pose.x_ = 0;
  //     pose.y_ = 0;
  //     pose.theta_ = 0;
  //   }
  //   catch (const std::exception& e)
  //   {
  //     std::cerr << e.what() << '\n';
  //     std::cerr << ros::Time::now() << " " << '\n';
  //   }
  // }
};

}  // namespace my_gmapping

#endif