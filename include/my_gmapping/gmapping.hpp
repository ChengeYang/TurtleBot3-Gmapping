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
    nh_.getParam("num_particles", num_particles_);

    // Publisher
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/my_gmapping/map", 1);
  }

  void laserScanCallback(const sensor_msgs::LaserScanPtr msg)
  {
  }

public:
  void init()
  {
    // Initialize all submodules through class_manager
    class_manager_ = std::make_shared<ClassManager>();
    class_manager_->initTransformManager(map_frame_, odom_frame_, base_frame_);
    class_manager_->initMotionModel();
    class_manager_->initMeasurementModel();
    class_manager_->initParticleFilter(num_particles_, 0.0, 0.0, 0.0, 0.0,
                                       map_resolution_, map_x_max_, map_x_min_,
                                       map_y_max_, map_y_min_);

    // Waiting for the initializations to finish
    ros::Duration(1).sleep();
  }

  void startSlam()
  {
    // Start subscribing to LaserScan topic /scan
    laser_scan_sub_ =
        nh_.subscribe("/scan", 1, &Gmapping::laserScanCallback, this);
  }
};

}  // namespace my_gmapping

#endif