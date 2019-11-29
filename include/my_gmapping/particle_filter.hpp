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
 * Particle Filter algorithm implementation.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include "my_gmapping/motion_model.hpp"
#include "my_gmapping/measurement_model.hpp"
#include "my_gmapping/particle.hpp"
#include "my_gmapping/mapper.hpp"
#include "my_gmapping/utils.hpp"

namespace my_gmapping
{
class ParticleFilter
{
public:
  // Particle set
  int num_particles_;
  std::vector<Particle> particle_set_;

  // Motion and Measurement models
  std::shared_ptr<MotionModel> motion_model_;
  std::shared_ptr<MeasurementModel> measurement_model_;

public:
  // Constructor
  ParticleFilter()
  {
  }

  ParticleFilter(std::shared_ptr<MotionModel> motion_model,
                 std::shared_ptr<MeasurementModel> measurement_model)
    : motion_model_(motion_model), measurement_model_(measurement_model)
  {
  }

  void initParticleSet(int num_particles, double timestamp, double x, double y,
                       double theta, double resolution, double x_max,
                       double x_min, double y_max, double y_min)
  {
    for (int i = 0; i < num_particles; i++)
    {
      Stamped2DPose init_pose(timestamp, x, y, theta);
      Mapper mapper(resolution, x_max, x_min, y_max, y_min);
      double weight = 1.0 / num_particles;
      particle_set_.push_back(Particle(init_pose, mapper, weight));
    }
  }

public:
  // Functions related with FastSLAM algorithm
  void motionUpdate()
  {
  }

  void scanMatch()
  {
  }

  void weightUpdate()
  {
  }

  void particleResample()
  {
  }

  void mapUpdate()
  {
  }
};

}  // namespace my_gmapping

#endif