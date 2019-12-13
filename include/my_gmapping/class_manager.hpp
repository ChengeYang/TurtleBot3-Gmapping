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
 * Manager node that is responsible for managing all submodules, including
 * calling the constructor and destructor of the classes.
 * This node takes arguments from gmapping.hpp and then initialize
 * corresponding class instances.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef CLASS_MANAGER_HPP
#define CLASS_MANAGER_HPP

#include <iostream>

#include <ros/ros.h>

#include "my_gmapping/transform_manager.hpp"
#include "my_gmapping/motion_model.hpp"
#include "my_gmapping/measurement_model.hpp"
#include "my_gmapping/scan_matcher.hpp"
#include "my_gmapping/particle_filter.hpp"

namespace my_gmapping
{
class ClassManager
{
public:
  // Submodules
  std::shared_ptr<TransformManager> transform_manager_;
  std::shared_ptr<MotionModel> motion_model_;
  std::shared_ptr<MeasurementModel> measurement_model_;
  std::shared_ptr<ScanMatcher> scan_matcher_;
  std::shared_ptr<ParticleFilter> particle_filter_;

public:
  // Constructor
  ClassManager()
  {
  }

public:
  void initTransformManager(std::string map_frame, std::string odom_frame,
                            std::string base_frame)
  {
    transform_manager_ =
        std::make_shared<TransformManager>(map_frame, odom_frame, base_frame);
  }

  void initMotionModel(double motion_noise_t_t, double motion_noise_x_y,
                       double motion_noise_t_r, double motion_noise_r_t,
                       double motion_noise_r_r)
  {
    motion_model_ = std::make_shared<MotionModel>(
        motion_noise_t_t, motion_noise_x_y, motion_noise_t_r, motion_noise_r_t,
        motion_noise_r_r);
  }

  void initMeasurementModel(double laser_range_max, double laser_range_min,
                            double l_free, double l_occupied,
                            double measurement_noise_range,
                            double measurement_noise_bearing)
  {
    measurement_model_ = std::make_shared<MeasurementModel>(
        laser_range_max, laser_range_min, l_free, l_occupied,
        measurement_noise_range, measurement_noise_bearing);
  }

  void initScanMatcher(int max_iter, int max_matched_pts, double epsilon,
                       double termination_error, double matching_factor)
  {
    scan_matcher_ = std::make_shared<ScanMatcher>(
        max_iter, max_matched_pts, epsilon, termination_error, matching_factor);
  }

  void initParticleFilter(int num_particles, double timestamp, double x,
                          double y, double theta, double resolution,
                          double x_max, double x_min, double y_max,
                          double y_min)
  {
    particle_filter_ = std::make_shared<ParticleFilter>(
        motion_model_, measurement_model_, scan_matcher_);
    particle_filter_->initParticleSet(num_particles, timestamp, x, y, theta,
                                      resolution, x_max, x_min, y_max, y_min);
  }
};

}  // namespace my_gmapping

#endif