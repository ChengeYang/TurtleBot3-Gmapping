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
#include <random>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include "my_gmapping/particle.hpp"
#include "my_gmapping/mapper.hpp"
#include "my_gmapping/motion_model.hpp"
#include "my_gmapping/measurement_model.hpp"
#include "my_gmapping/scan_matcher.hpp"
#include "my_gmapping/utils.hpp"

namespace my_gmapping
{
class ParticleFilter
{
public:
  // Particle set
  int num_particles_;
  std::vector<Particle> particle_set_;

  // Motion, Measurement model and Scan Matcher
  std::shared_ptr<MotionModel> motion_model_;
  std::shared_ptr<MeasurementModel> measurement_model_;
  std::shared_ptr<ScanMatcher> scan_matcher_;

  // Robot pose and map estimate
  Stamped2DPose pose_;
  Mapper mapper_;

  // Random number generater
  std::random_device random_device_;
  std::mt19937 random_generator_;

public:
  // Constructor
  ParticleFilter()
  {
  }

  ParticleFilter(std::shared_ptr<MotionModel> motion_model,
                 std::shared_ptr<MeasurementModel> measurement_model,
                 std::shared_ptr<ScanMatcher> scan_matcher)
    : motion_model_(motion_model)
    , measurement_model_(measurement_model)
    , scan_matcher_(scan_matcher)
    , random_generator_(random_device_())
  {
  }

  void initParticleSet(int num_particles, double timestamp, double x, double y,
                       double theta, double resolution, double x_max,
                       double x_min, double y_max, double y_min)
  {
    // Initialize particle set
    Stamped2DPose init_pose(timestamp, x, y, theta);
    Mapper init_mapper(resolution, x_max, x_min, y_max, y_min);
    double weight = 1.0 / num_particles;
    for (int i = 0; i < num_particles; i++)
    {
      particle_set_.push_back(Particle(init_pose, init_mapper, weight));
    }

    // Initialize member variables
    num_particles_ = num_particles;
    pose_ = init_pose;
    mapper_ = init_mapper;
  }

  void update(const sensor_msgs::LaserScan& msg,
              const Stamped2DPose& pre_odom_pose,
              const Stamped2DPose& cur_odom_pose)
  {
    LaserScanReading reading = measurement_model_->processScan(msg);
    if (reading.ranges_.size() < 1)
      return;

    for (size_t i = 0; i < particle_set_.size(); i++)
    {
      motionUpdate(particle_set_[i], pre_odom_pose, cur_odom_pose);
      scanMatch(particle_set_[i], reading);
      weightUpdate(particle_set_[i]);
      mapUpdate(particle_set_[i], reading);
    }
    stateUpdate();
    particleResample();
  }

public:
  // Functions related with FastSLAM algorithm
  void motionUpdate(Particle& particle, const Stamped2DPose& pre_odom_pose,
                    const Stamped2DPose& cur_odom_pose)
  {
    motion_model_->sampleMotionModel(particle, pre_odom_pose, cur_odom_pose);
  }

  void scanMatch(Particle& particle, const LaserScanReading& reading)
  {
    // Convert data into 2D (x, y) pointcloud
    PointCloudXY model, data;
    measurement_model_->generatePointCloudXY(particle, model);
    measurement_model_->generatePointCloudXY(particle, reading, data);

    // ICP
    Stamped2DPose transform;
    scan_matcher_->icpMatching(model, data, transform);
  }

  void weightUpdate(Particle& particle)
  {
    // Normalization
    double weight_sum = 0.0;
    for (size_t i = 0; i < particle_set_.size(); i++)
    {
      weight_sum += particle_set_[i].weight_;
    }
    if (weight_sum < 0.000001)
    {
      for (size_t i = 0; i < particle_set_.size(); i++)
        particle_set_[i].weight_ = 1.0 / num_particles_;
    }
    else
    {
      for (size_t i = 0; i < particle_set_.size(); i++)
        particle_set_[i].weight_ /= weight_sum;
    }
  }

  void mapUpdate(Particle& particle, const LaserScanReading& reading)
  {
    measurement_model_->integrateScan(particle, reading);
  }

  void stateUpdate()
  {
    // Update current estimate to the best particle (highest weight)
    int particle_idx = 0;
    double max_weight = 0;
    for (size_t i = 0; i < particle_set_.size(); i++)
    {
      if (particle_set_[i].weight_ > max_weight)
      {
        particle_idx = i;
        max_weight = particle_set_[i].weight_;
      }
    }
    pose_ = particle_set_[particle_idx].cur_pose_;
    mapper_ = particle_set_[particle_idx].mapper_;

    // // Update current estimate to the average of all particles
    // Stamped2DPose pose = particle_set_[0].cur_pose_;
    // Mapper mapper = particle_set_[0].mapper_;
    // pose.x_ = 0;
    // pose.y_ = 0;
    // pose.theta_ = 0;
    // for (size_t i = 0; i < mapper.map_.size(); i++)
    // {
    //   mapper.map_[i] = 0;
    // }

    // for (size_t i = 0; i < num_particles_; i++)
    // {
    //   pose.x_ += particle_set_[i].cur_pose_.x_;
    //   pose.y_ += particle_set_[i].cur_pose_.y_;
    //   pose.theta_ += particle_set_[i].cur_pose_.theta_;
    //   for (size_t j = 0; j < mapper.map_.size(); j++)
    //   {
    //     mapper.map_[j] += particle_set_[i].mapper_.map_[j];
    //   }
    // }

    // pose.x_ /= num_particles_;
    // pose.y_ /= num_particles_;
    // pose.theta_ /= num_particles_;
    // normalizeTheta(pose.theta_);
    // for (size_t i = 0; i < mapper.map_.size(); i++)
    // {
    //   mapper.map_[i] /= num_particles_;
    // }

    // pose_ = pose;
    // mapper_ = mapper;
  }

  void particleResample()
  {
    // Resample criteria
    // 0.5 is the threshold proposed by Cyrill Stachniss
    // double T = 0.5 * num_particles_;
    // double Neff = 0.0;
    // for (size_t i = 0; i < particle_set_.size(); i++)
    // {
    //   Neff += particle_set_[i].weight_ * particle_set_[i].weight_;
    // }
    // Neff = 1.0 / Neff;
    // if (Neff >= T)
    //   return;

    // Generate random indexes according to weights
    std::vector<double> weights;
    for (size_t i = 0; i < particle_set_.size(); i++)
    {
      weights.push_back(particle_set_[i].weight_);
    }
    std::discrete_distribution<> distribution(weights.begin(), weights.end());
    std::vector<int> indexes;
    for (size_t i = 0; i < num_particles_; i++)
    {
      int index = distribution(random_generator_);
      indexes.push_back(index);
    }

    // Resample
    std::vector<Particle> new_particle_set;
    for (auto index : indexes)
    {
      Particle new_particle = particle_set_[index];
      new_particle_set.push_back(new_particle);
    }
    particle_set_ = std::move(new_particle_set);
  }
};

}  // namespace my_gmapping

#endif