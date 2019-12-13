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

#define BEAM_ANGLE 0.017453292519943

#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Core>

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
  // Measurement model noise
  double measurement_noise_range_;
  double measurement_noise_bearing_;

public:
  // Constructor
  MeasurementModel()
  {
  }

  MeasurementModel(double laser_range_max, double laser_range_min,
                   double l_free, double l_occupied,
                   double measurement_noise_range,
                   double measurement_noise_bearing)
    : laser_range_max_(laser_range_max)
    , laser_range_min_(laser_range_min)
    , l_free_(l_free)
    , l_occupied_(l_occupied)
    , measurement_noise_range_(measurement_noise_range)
    , measurement_noise_bearing_(measurement_noise_bearing)
  {
  }

public:  // Funtions for process LaserScan sensor data
  // Transfer sensor_msgs::LaserScan to LaserScanReading datatype
  // Here we assume that scan has 360 beams with angle of 360 degrees
  LaserScanReading processScan(const sensor_msgs::LaserScan& msg)
  {
    LaserScanReading reading;

    for (size_t i = 0; i < msg.ranges.size(); i++)
    {
      // Filter beam that is outside the range
      if (laser_range_min_ <= msg.ranges[i] && msg.range_min <= msg.ranges[i] &&
          msg.ranges[i] <= laser_range_max_ && msg.ranges[i] <= msg.range_max)
      {
        reading.ranges_.push_back(msg.ranges[i]);
        reading.bearings_.push_back(0.0 + i * BEAM_ANGLE);
      }
      else
      {
        reading.ranges_.push_back(laser_range_max_);
        reading.bearings_.push_back(0.0 + i * BEAM_ANGLE);
      }
    }

    return reading;
  }

  // Generate expected laser reading from given pose and map
  // Here we assume 360 beam angles in range [0, 2*pi]
  LaserScanReading generateLaserReading(const StampedPose2D& pose,
                                        const Mapper& mapper)
  {
    LaserScanReading reading;

    // Table to record the closest range at each angle
    std::vector<double> ranges(360, laser_range_max_);

    double x_start = pose.x_ - laser_range_max_;
    double x_end = pose.x_ + laser_range_max_;
    double y_start = pose.y_ - laser_range_max_;
    double y_end = pose.y_ + laser_range_max_;
    double step = mapper.resolution_;

    for (double x = x_start; x < x_end; x += step)
    {
      for (double y = y_start; y < y_end; y += step)
      {
        // Get grid vector index
        int index = mapper.xyToVectorIndex(x, y);
        if (index < 0 or mapper.map_[index] <= 0)
          continue;

        // Get (x, y) coordinate of map grid center
        double x_center, y_center;
        mapper.vectorIndexToXY(index, x_center, y_center);

        // Get expected range and bearing value
        double range = sqrt((x_center - pose.x_) * (x_center - pose.x_) +
                            (y_center - pose.y_) * (y_center - pose.y_));
        double bearing =
            atan2(y_center - pose.y_, x_center - pose.x_) - pose.theta_;
        normalizeTheta2(bearing);

        // Find nearest beam index
        int beam_index = std::floor(bearing / BEAM_ANGLE);
        double res = bearing - beam_index * BEAM_ANGLE;
        if (res > BEAM_ANGLE / 2.0)
          beam_index += 1;

        if (range < ranges[beam_index])
          ranges[beam_index] = range;
      }
    }

    for (size_t i = 0; i < ranges.size(); i++)
    {
      reading.ranges_.push_back(ranges[i]);
      reading.bearings_.push_back(0.0 + i * BEAM_ANGLE);
    }

    return reading;
  }

  // Generate ICP model from a particle's pose and mapper
  std::vector<Eigen::Vector3d> generateICPModel(const StampedPose2D& pose,
                                                const Mapper& mapper)
  {
    std::vector<Eigen::Vector3d> points;

    double x_start = pose.x_ - laser_range_max_;
    double x_end = pose.x_ + laser_range_max_;
    double y_start = pose.y_ - laser_range_max_;
    double y_end = pose.y_ + laser_range_max_;
    double step = mapper.resolution_;

    for (double x = x_start; x < x_end; x += step)
    {
      for (double y = y_start; y < y_end; y += step)
      {
        int index = mapper.xyToVectorIndex(x, y);
        if (index < 0 or mapper.map_[index] <= 0)
          continue;

        double x_center, y_center;
        mapper.vectorIndexToXY(index, x_center, y_center);

        points.push_back(Eigen::Vector3d(x_center, y_center, 0.0));
      }
    }

    return points;
  }

  // Generate ICP data from particle pose and laser scan data
  std::vector<Eigen::Vector3d> generateICPData(const StampedPose2D& pose,
                                               const LaserScanReading& reading)
  {
    std::vector<Eigen::Vector3d> points;

    for (size_t i = 0; i < reading.ranges_.size(); i++)
    {
      if (reading.ranges_[i] >= laser_range_max_)
        continue;

      double theta = pose.theta_ + reading.bearings_[i];
      double x = pose.x_ + std::cos(theta) * reading.ranges_[i];
      double y = pose.y_ + std::sin(theta) * reading.ranges_[i];

      points.push_back(Eigen::Vector3d(x, y, 0.0));
    }

    return points;
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
      if (reading.ranges_[i] >= laser_range_max_)
        continue;

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
  // Measurement Model
  // Compute the likelihood of scan P(Zt|Xt, m)
  double computeScanLikelihood(const StampedPose2D& pose, const Mapper& mapper,
                               const LaserScanReading& reading_laser)
  {
    // Expected reading generated from robot pose and map
    LaserScanReading reading_map = generateLaserReading(pose, mapper);

    // Assume beam is independent to each other
    // Assume range and bearing are independent
    // Select 36 beams among 360
    double p = 1.0;
    for (size_t i = 0; i < reading_map.ranges_.size(); i += 5)
    {
      double difference;

      if ((reading_map.ranges_[i] >= laser_range_max_ and
           reading_laser.ranges_[i] < laser_range_max_) or
          (reading_map.ranges_[i] < laser_range_max_ and
           reading_laser.ranges_[i] >= laser_range_max_))
        difference = laser_range_max_ + measurement_noise_range_;
      else
        difference = fabs(reading_map.ranges_[i] - reading_laser.ranges_[i]);

      p *= computeGaussianLikelihood(difference, 0, measurement_noise_range_);
    }

    return p;
  }

  // Inverse measurement model: Probabilistic Robotics P288, Table 9.2
  // Compute the occupancy grid status given robot pose, map, laser
  // scan data and grid index of map vector
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