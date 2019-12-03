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
 * Functions for measurement model and range sensor.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP

#include <iostream>
#include <cmath>

#include <sensor_msgs/LaserScan.h>

#include "my_gmapping/particle.hpp"
#include "my_gmapping/utils.hpp"

namespace my_gmapping
{
class MeasurementModel
{
public:
  double laser_range_max_;

public:
  MeasurementModel()
  {
  }

  MeasurementModel(double laser_range_max) : laser_range_max_(laser_range_max)
  {
  }

public:
  void addScanToMap(Particle& particle, const sensor_msgs::LaserScan& msg)
  {
    // Get bearings
    std::vector<double> bearings;
    getLaserScanBearings(msg, bearings);

    // Map to record update status of each grid
    // -1 free, 0 no update, 1 occupied
    std::vector<int> update_queue(particle.mapper_.map_.size(), 0);
    for (size_t i = 0; i < msg.ranges.size(); i++)
    {
      if (msg.ranges[i] > laser_range_max_)
        continue;

      int index = getLaserScanIndex(particle, msg.ranges[i], bearings[i]);
      update_queue[index] = 1;
    }

    // Update map
    for (size_t i = 0; i < update_queue.size(); i++)
    {
      if (update_queue[i] == 1)
      {
        particle.mapper_.map_[i] += 1.0;
        continue;
      }

      if (update_queue[i] == -1)
      {
        particle.mapper_.map_[i] -= 1.0;
        continue;
      }
    }
  }

public:
  // Process laser scan input
  void getLaserScanBearings(const sensor_msgs::LaserScan& msg,
                            std::vector<double>& bearings)
  {
    for (size_t i = 0; i < msg.ranges.size(); i++)
    {
      bearings.push_back(msg.angle_min + i * msg.angle_increment);
    }
  }

  // Return the index in vector map given robot pose and measurement
  int getLaserScanIndex(const Particle& particle, double range, double bearing)
  {
    double angle = particle.cur_pose_.theta_ + bearing;
    normalizeTheta(angle);

    double x = particle.cur_pose_.x_ + range * std::cos(angle);
    double y = particle.cur_pose_.y_ + range * std::sin(angle);

    int index = particle.mapper_.posToIndex(x, y);

    return index;
  }
};

}  // namespace my_gmapping

#endif