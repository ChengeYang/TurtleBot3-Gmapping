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
 * Main node responsible for providing ROS interface and controlling the
 * algorithm workflow. The main function are as follows:
 * 1. Manage ROS topics and services.
 * 2. Implement callbacks for ROS subscribers.
 * 3. Read in launch file parameters.
 * 4. Control the workflow of the algorithm.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef GMAPPING_HPP
#define GMAPPING_HPP

#include <iostream>

#include <ros/ros.h>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include <tf2/LinearMath/Quaternion.h>

#include "my_gmapping/class_manager.hpp"

namespace my_gmapping
{
class Gmapping
{
public:
  // ROS NodeHandler
  ros::NodeHandle nh_;

  // Subscribers
  // /scan: [sensor_msgs/LaserScan]
  ros::Subscriber laser_scan_sub_;

  // Publishers
  // /map: [nav_msgs/OccupancyGrid]
  ros::Publisher map_pub_;
  ros::Publisher particles_marker_pub_;

public:
  // Launch file parameters

  // /map: map frame fixed at (0, 0)
  // /map  to /odom:           drifting accumulated by odometry data
  // /odom to /base_footprint: odometry data published from robot
  // /map  to /base_footprint: robot pose obtained from particle filter
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  // Map is centered at (0, 0)
  double map_resolution_;
  double map_x_max_;
  double map_x_min_;
  double map_y_max_;
  double map_y_min_;

  // Motion model noise
  double motion_noise_t_t_;  // Noise in x or y due to change in x or y
  double motion_noise_x_y_;  // Noise in x or y due to change in y or x
  double motion_noise_t_r_;  // Noise in x or y due to change in theta
  double motion_noise_r_t_;  // Noise in theta due to change in y or x
  double motion_noise_r_r_;  // Noise in theta due to change in theta

  // Measurement model
  double laser_range_max_;
  double laser_range_min_;
  // Log odds information for initial, free and occupied grid
  double l_free_;
  double l_occupied_;
  // Measurement model noise
  double measurement_noise_range_;
  double measurement_noise_bearing_;

  // Scan Matcher
  int max_iter_;
  int max_matched_pts_;
  double epsilon_;
  double termination_error_;
  double matching_factor_;

  // Particle Filter
  int num_particles_;

public:
  // Submodule manager
  std::shared_ptr<ClassManager> class_manager_;

public:
  // Constructor
  Gmapping()
  {
    // Private NodeHandler
    nh_ = ros::NodeHandle("~");

    // Read in launch file parameters
    nh_.getParam("map_frame", map_frame_);
    nh_.getParam("odom_frame", odom_frame_);
    nh_.getParam("base_frame", base_frame_);
    nh_.getParam("map_resolution", map_resolution_);
    nh_.getParam("map_x_max", map_x_max_);
    nh_.getParam("map_x_min", map_x_min_);
    nh_.getParam("map_y_max", map_y_max_);
    nh_.getParam("map_y_min", map_y_min_);
    nh_.getParam("motion_noise_t_t", motion_noise_t_t_);
    nh_.getParam("motion_noise_x_y", motion_noise_x_y_);
    nh_.getParam("motion_noise_t_r", motion_noise_t_r_);
    nh_.getParam("motion_noise_r_t", motion_noise_r_t_);
    nh_.getParam("motion_noise_r_r", motion_noise_r_r_);
    nh_.getParam("laser_range_max", laser_range_max_);
    nh_.getParam("laser_range_min", laser_range_min_);
    nh_.getParam("l_free", l_free_);
    nh_.getParam("l_occupied", l_occupied_);
    nh_.getParam("measurement_noise_range", measurement_noise_range_);
    nh_.getParam("measurement_noise_bearing", measurement_noise_bearing_);
    nh_.getParam("max_iter", max_iter_);
    nh_.getParam("max_matched_pts", max_matched_pts_);
    nh_.getParam("epsilon", epsilon_);
    nh_.getParam("termination_error", termination_error_);
    nh_.getParam("matching_factor", matching_factor_);
    nh_.getParam("num_particles", num_particles_);

    // Publisher
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/my_gmapping/map", 1);
    particles_marker_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/my_gmapping/particles", 1);
  }

  void init()
  {
    // Initialize all submodules through class_manager
    class_manager_ = std::make_shared<ClassManager>();
    class_manager_->initTransformManager(map_frame_, odom_frame_, base_frame_);
    class_manager_->initMotionModel(motion_noise_t_t_, motion_noise_x_y_,
                                    motion_noise_t_r_, motion_noise_r_t_,
                                    motion_noise_r_r_);
    class_manager_->initMeasurementModel(
        laser_range_max_, laser_range_min_, l_free_, l_occupied_,
        measurement_noise_range_, measurement_noise_bearing_);
    class_manager_->initScanMatcher(max_iter_, max_matched_pts_, epsilon_,
                                    termination_error_, matching_factor_);
    class_manager_->initParticleFilter(
        num_particles_,
        class_manager_->transform_manager_->cur_odom_pose_.timestamp_,
        class_manager_->transform_manager_->cur_odom_pose_.x_,
        class_manager_->transform_manager_->cur_odom_pose_.y_,
        class_manager_->transform_manager_->cur_odom_pose_.theta_,
        map_resolution_, map_x_max_, map_x_min_, map_y_max_, map_y_min_);

    // Waiting for the initializations to finish
    ros::Duration(1).sleep();
  }

  void startSlam()
  {
    // Start subscribing to LaserScan topic /scan
    laser_scan_sub_ =
        nh_.subscribe("/scan", 1, &Gmapping::laserScanCallback, this);

    // Publish loop
    ros::Rate rate(30);
    while (ros::ok())
    {
      publishData();
      ros::spinOnce();
      rate.sleep();
    }
  }

public:
  void laserScanCallback(const sensor_msgs::LaserScan& msg)
  {
    // Read odometry data from tf
    if (!class_manager_->transform_manager_->getOdometryData())
      return;

    // Conduct update in Particle Filter
    class_manager_->particle_filter_->update(
        msg, class_manager_->transform_manager_->pre_odom_pose_,
        class_manager_->transform_manager_->cur_odom_pose_);
  }

  void publishData()
  {
    // Publish robot pose estimate to tf
    class_manager_->transform_manager_->publishMapToOdomTransform(
        class_manager_->particle_filter_->pose_);

    // Publish map estimate to /my_gmapping/map
    publishMap(class_manager_->particle_filter_->mapper_);

    // Publish particle cloud to /my_gmapping/particles
    publishParticles(class_manager_->particle_filter_->particle_set_);
  }

  void publishMap(const Mapper& mapper)
  {
    nav_msgs::OccupancyGrid map_msg;

    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_;

    map_msg.info.resolution = mapper.resolution_;
    map_msg.info.width = mapper.x_size_;
    map_msg.info.height = mapper.y_size_;
    map_msg.info.origin.position.x = mapper.x_min_;
    map_msg.info.origin.position.y = mapper.y_min_;
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.x = 0.0;
    map_msg.info.origin.orientation.y = 0.0;
    map_msg.info.origin.orientation.z = 0.0;
    map_msg.info.origin.orientation.w = 0.0;

    map_msg.data.resize(map_msg.info.width * map_msg.info.height);
    for (size_t i = 0; i < mapper.map_.size(); i++)
    {
      // Free
      if (mapper.map_[i] < 0.0)
        map_msg.data[i] = -1;
      // Unknown
      else if (mapper.map_[i] < 0.25)
        map_msg.data[i] = 0;
      // Obstacle
      else
        map_msg.data[i] = 100;
    }

    map_pub_.publish(map_msg);
  }

  void publishParticles(const std::vector<Particle>& particle_set)
  {
    visualization_msgs::Marker particles_marker;

    particles_marker.header.stamp = ros::Time::now();
    particles_marker.header.frame_id = map_frame_;

    particles_marker.ns = "my_gmapping";
    particles_marker.id = 0;
    particles_marker.type = visualization_msgs::Marker::POINTS;
    particles_marker.action = visualization_msgs::Marker::ADD;
    particles_marker.pose.orientation.w = 1.0;

    particles_marker.scale.x = 0.02;
    particles_marker.scale.y = 0.02;

    particles_marker.color.r = 1.0;
    particles_marker.color.a = 1.0;

    for (size_t i = 0; i < particle_set.size(); i++)
    {
      geometry_msgs::Point point_msg;

      point_msg.x = particle_set[i].cur_pose_.x_;
      point_msg.y = particle_set[i].cur_pose_.y_;
      point_msg.z = 0.0;

      particles_marker.points.push_back(point_msg);
    }

    particles_marker_pub_.publish(particles_marker);
  }
};

}  // namespace my_gmapping

#endif