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
 * Utilities file containing datatypes, utility functions, etc.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef UTILS_HPP
#define UTILS_HPP

#include <math.h>

#include <sensor_msgs/LaserScan.h>

namespace my_gmapping
{
// Class for representing robot pose [timestamp, x, y, theta]
class Stamped2DPose
{
public:
  double timestamp_;
  double x_;
  double y_;
  double theta_;  // [-pi, pi]

public:
  Stamped2DPose()
  {
  }

  Stamped2DPose(double timestamp, double x, double y, double theta)
    : timestamp_(timestamp), x_(x), y_(y), theta_(theta)
  {
  }
};

// Class to process and store readings from laser scan input
class LaserScanReading
{
public:
  std::vector<double> ranges_;
  std::vector<double> bearings_;

public:
  LaserScanReading()
  {
  }

  LaserScanReading(const sensor_msgs::LaserScan& msg, double laser_range_max,
                   double laser_range_min)
  {
    for (size_t i = 0; i < msg.ranges.size(); i++)
    {
      if (laser_range_min <= msg.ranges[i] && msg.range_min <= msg.ranges[i] &&
          msg.ranges[i] <= laser_range_max && msg.ranges[i] <= msg.range_max)
      {
        ranges_.push_back(msg.ranges[i]);
        bearings_.push_back(msg.angle_min + i * msg.angle_increment);
      }
    }
  }
};

// Class to record 2D (x, y) PointCloud in active area for scan matching
class PointCloudXY
{
public:
  int size_;
  std::vector<double> xs_;
  std::vector<double> ys_;

public:
  PointCloudXY()
  {
  }

  void clearData()
  {
    size_ = 0;
    xs_.clear();
    ys_.clear();
  }
};

// Normalize angle to stay in [-pi, +pi]
double normalizeTheta(double& theta)
{
  if (theta > M_PI)
    theta -= 2 * M_PI;
  else if (theta < -M_PI)
    theta += 2 * M_PI;
}

// Normalize angle to stay in [0, +2*pi]
double normalizeTheta2(double& theta)
{
  if (theta > 2 * M_PI)
    theta -= 2 * M_PI;
  else if (theta < 0)
    theta += 2 * M_PI;
}

}  // namespace my_gmapping

#endif