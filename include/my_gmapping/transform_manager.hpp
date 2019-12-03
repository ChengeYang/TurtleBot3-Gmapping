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
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "my_gmapping/utils.hpp"

namespace my_gmapping
{
class TransformManager
{
private:
  // TF subscriber and publisher
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Frame names
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

public:
  // Current and previous odometry data
  geometry_msgs::TransformStamped pre_odom_transform_;
  Stamped2DPose pre_odom_pose_;
  geometry_msgs::TransformStamped cur_odom_transform_;
  Stamped2DPose cur_odom_pose_;

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
    // Get initial transform between /odom and /base
    while (!this->getOdometryData())
    {
    }
  }

public:
  // Read odometry data from /tf and convert to Stamped2DPose
  bool getOdometryData()
  {
    geometry_msgs::TransformStamped temp_transform;
    Stamped2DPose temp_pose;

    try
    {
      temp_transform = tf_buffer_.lookupTransform(
          odom_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
      convert(temp_transform, temp_pose);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }

    pre_odom_transform_ = cur_odom_transform_;
    pre_odom_pose_ = cur_odom_pose_;
    cur_odom_transform_ = temp_transform;
    cur_odom_pose_ = temp_pose;

    return true;
  }

  // Publish /map to /odom transform to tf given current robot pose estimate
  // from Particle Filter
  void publishMapToOdomTransform(Stamped2DPose pose)
  {
    // std::cerr << pose.timestamp_ << " " << pose.x_ << " " << pose.y_ << " "
    //           << pose.theta_ << std::endl;
    pose.x_ -= cur_odom_pose_.x_;
    pose.y_ -= cur_odom_pose_.y_;
    pose.theta_ -= cur_odom_pose_.theta_;
    normalizeTheta(pose.theta_);

    geometry_msgs::TransformStamped transform_stamped;
    convert(pose, transform_stamped);

    tf_broadcaster_.sendTransform(transform_stamped);
  }

public:
  // Convertion functions between datatypes
  void convert(geometry_msgs::TransformStamped transform_stamped,
               Stamped2DPose& pose)
  {
    pose.timestamp_ = transform_stamped.header.stamp.toSec();
    pose.x_ = transform_stamped.transform.translation.x;
    pose.y_ = transform_stamped.transform.translation.y;

    tf2::Quaternion q(transform_stamped.transform.rotation.x,
                      transform_stamped.transform.rotation.y,
                      transform_stamped.transform.rotation.z,
                      transform_stamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose.theta_ = yaw;
  }

  void convert(Stamped2DPose pose,
               geometry_msgs::TransformStamped& transform_stamped)
  {
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = map_frame_;

    transform_stamped.child_frame_id = odom_frame_;

    transform_stamped.transform.translation.x = pose.x_;
    transform_stamped.transform.translation.y = pose.y_;
    transform_stamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta_);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
  }
};

}  // namespace my_gmapping

#endif