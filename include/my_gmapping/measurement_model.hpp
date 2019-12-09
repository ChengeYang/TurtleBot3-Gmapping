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

#ifndef MEASUREMENT_MODEL_HPP
#define MEASUREMENT_MODEL_HPP

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
  // Laser scan range sensor range
  double laser_range_max_;
  double laser_range_min_;
  // Log odds information for free and occupied grid
  double l_free_;
  double l_occupied_;

public:
  MeasurementModel()
  {
  }

  MeasurementModel(double laser_range_max, double laser_range_min,
                   double l_free, double l_occupied)
    : laser_range_max_(laser_range_max)
    , laser_range_min_(laser_range_min)
    , l_free_(l_free)
    , l_occupied_(l_occupied)
  {
  }

public:  // Funtions for process LaserScan sensor data
  // Transfer sensor_msgs::LaserScan to LaserScanReading datatype
  LaserScanReading processScan(const sensor_msgs::LaserScan& msg)
  {
    return LaserScanReading(msg, laser_range_max_, laser_range_min_);
  }

  // Generate 2D point cloud from particle and the map it brings (as ICP model)
  void generatePointCloudXY(const Particle& particle, PointCloudXY& point_cloud)
  {
    point_cloud.xs_.clear();
    point_cloud.ys_.clear();

    double x_start = particle.cur_pose_.x_ - laser_range_max_;
    double x_end = particle.cur_pose_.x_ + laser_range_max_;
    double y_start = particle.cur_pose_.y_ - laser_range_max_;
    double y_end = particle.cur_pose_.y_ + laser_range_max_;
    double step = particle.mapper_.resolution_;

    for (double x = x_start; x < x_end; x += step)
    {
      for (double y = y_start; y < y_end; y += step)
      {
        int index = particle.mapper_.xyToVectorIndex(x, y);
        if (index < 0 or particle.mapper_.map_[index] <= 0)
          continue;

        double x_real, y_real;
        particle.mapper_.vectorIndexToXY(index, x_real, y_real);
        point_cloud.xs_.push_back(x_real);
        point_cloud.ys_.push_back(y_real);
      }
    }
    point_cloud.size_ = point_cloud.xs_.size();
  }

  // Generate 2D point cloud from particle and laser scan data (as ICP data)
  void generatePointCloudXY(const Particle& particle,
                            const LaserScanReading& reading,
                            PointCloudXY& point_cloud)
  {
    point_cloud.xs_.clear();
    point_cloud.ys_.clear();

    for (size_t i = 0; i < reading.ranges_.size(); i++)
    {
      double theta = particle.cur_pose_.theta_ + reading.bearings_[i];
      double x_real =
          particle.cur_pose_.x_ + std::cos(theta) * reading.ranges_[i];
      double y_real =
          particle.cur_pose_.y_ + std::sin(theta) * reading.ranges_[i];

      point_cloud.xs_.push_back(x_real);
      point_cloud.ys_.push_back(y_real);
    }
    point_cloud.size_ = point_cloud.xs_.size();
  }

public:  // Functions for map update
  // Integrate scan into Occupancy Grid map
  void integrateScan(Particle& particle, const LaserScanReading& reading)
  {
    // Map to record update status of each grid
    // -1 free, 0 no update, 1 occupied
    std::vector<int> update_queue(particle.mapper_.map_.size(), 0);
    for (size_t i = 0; i < reading.ranges_.size(); i++)
    {
      // Free grid: waypoints in the laser route
      std::vector<int> idx_free =
          getFreeGridIndex(particle, reading.ranges_[i], reading.bearings_[i]);
      for (size_t i = 0; i < idx_free.size(); i++)
        update_queue[idx_free[i]] = -1;

      // Occupied grid: laser scan reading
      std::vector<int> idx_occupied = getOccupiedGridIndex(
          particle, reading.ranges_[i], reading.bearings_[i]);
      for (size_t i = 0; i < idx_occupied.size(); i++)
        update_queue[idx_occupied[i]] = 1;
    }

    // Update map
    for (size_t i = 0; i < update_queue.size(); i++)
    {
      if (update_queue[i] == -1)
        particle.mapper_.map_[i] += l_free_;
      else if (update_queue[i] == 1)
        particle.mapper_.map_[i] += l_occupied_;
    }
  }

  // Return the occupied grid index of map vector given robot pose and
  // measurement
  std::vector<int> getOccupiedGridIndex(const Particle& particle, double range,
                                        double bearing)
  {
    double angle = particle.cur_pose_.theta_ + bearing;
    normalizeTheta(angle);

    double x = particle.cur_pose_.x_ + range * std::cos(angle);
    double y = particle.cur_pose_.y_ + range * std::sin(angle);

    std::vector<int> idx_occupied;
    int index = particle.mapper_.xyToVectorIndex(x, y);
    if (index > -1)
      idx_occupied.push_back(index);

    return idx_occupied;
  }

  // Return the free grid indexes of map vector given robot pose and
  // measurement (Bresenham's line algorithm)
  std::vector<int> getFreeGridIndex(const Particle& particle, double range,
                                    double bearing)
  {
    double angle = particle.cur_pose_.theta_ + bearing;
    normalizeTheta(angle);

    double x_start = particle.cur_pose_.x_;
    double y_start = particle.cur_pose_.y_;
    double x_end = particle.cur_pose_.x_ + range * std::cos(angle);
    double y_end = particle.cur_pose_.y_ + range * std::sin(angle);
    double delta_x = x_end - x_start;
    double delta_y = y_end - y_start;
    double resolution = particle.mapper_.resolution_;

    std::vector<int> idx_free;
    if (delta_x >= delta_y)
    {
      double slope = fabs(delta_y / delta_x);
      if (delta_x > 0)
      {
        for (double x = x_start, y = y_start; x < x_end;
             x += resolution, y += copysign(resolution, delta_y) * slope)
        {
          int index = particle.mapper_.xyToVectorIndex(x, y);
          if (index > -1)
            idx_free.push_back(index);
        }
      }
      else
      {
        for (double x = x_start, y = y_start; x > x_end;
             x -= resolution, y += copysign(resolution, delta_y) * slope)
        {
          int index = particle.mapper_.xyToVectorIndex(x, y);
          if (index > -1)
            idx_free.push_back(index);
        }
      }
    }
    else
    {
      double slope = fabs(delta_x / delta_y);
      if (delta_y > 0)
      {
        for (double x = x_start, y = y_start; y < y_end;
             x += copysign(resolution, delta_x) * slope, y += resolution)
        {
          int index = particle.mapper_.xyToVectorIndex(x, y);
          if (index > -1)
            idx_free.push_back(index);
        }
      }
      else
      {
        for (double x = x_start, y = y_start; y > y_end;
             x += copysign(resolution, delta_x) * slope, y -= resolution)
        {
          int index = particle.mapper_.xyToVectorIndex(x, y);
          if (index > -1)
            idx_free.push_back(index);
        }
      }
    }

    return idx_free;
  }

public:  // Functions for measurement model
  // Compute the occupancy grid status given robot pose, map, laser
  // scan data and grid index of map vector
  // See Probabilistic Robotics P288, Table 9.2
  double inverseRangeSensorModel(const Particle& particle,
                                 const LaserScanReading& reading, int index)
  {
    // Laser beam width and angle allowed for matching
    double alpha = particle.mapper_.resolution_ * 4;  // 3 OccupancyGrid
    double beta = M_PI / 180 * 5;                     // 5 Degree

    // Get the grid center coordinate
    double x_grid, y_grid;
    particle.mapper_.vectorIndexToXY(index, x_grid, y_grid);

    // Compute expected measurement
    double delta_x = x_grid - particle.cur_pose_.x_;
    double delta_y = y_grid - particle.cur_pose_.y_;
    double r = sqrt(delta_x * delta_x + delta_y * delta_y);
    double theta = atan2(delta_y, delta_x) - particle.cur_pose_.theta_;
    normalizeTheta2(theta);

    // Find laser beam that is closest to expected bearing
    int k = 0;
    double difference = 2 * M_PI;
    for (size_t i = 0; i < reading.bearings_.size(); i++)
    {
      if (fabs(reading.bearings_[i] - theta) < difference)
      {
        k = i;
        difference = fabs(reading.bearings_[i] - theta);
      }
    }
    double range = reading.ranges_[k];
    double bearing = reading.bearings_[k];

    // Return grid information update
    if (r > std::min(laser_range_max_, range + alpha / 2) or
        fabs(theta - bearing) > beta / 2)
      return 0.0;
    else if (range < laser_range_max_ and fabs(r - range) < alpha / 2)
      return l_occupied_;
    else if (r < range)
      return l_free_;
  }
};

}  // namespace my_gmapping

#endif