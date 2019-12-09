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
 * Functions for motion model and odometry sensor.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <iostream>
#include <random>

#include "my_gmapping/particle.hpp"
#include "my_gmapping/utils.hpp"

namespace my_gmapping
{
class MotionModel
{
public:
  // Motion model noise
  double noise_t_t_;  // Noise in x or y due to change in x or y
  double noise_x_y_;  // Noise in x or y due to change in y or x
  double noise_t_r_;  // Noise in x or y due to change in theta
  double noise_r_t_;  // Noise in theta due to change in y or x
  double noise_r_r_;  // Noise in theta due to change in theta

  // Random generator
  std::default_random_engine generator_;

public:
  MotionModel(double motion_noise_t_t, double motion_noise_x_y,
              double motion_noise_t_r, double motion_noise_r_t,
              double motion_noise_r_r)
    : noise_t_t_(motion_noise_t_t)
    , noise_x_y_(motion_noise_x_y)
    , noise_t_r_(motion_noise_t_r)
    , noise_r_t_(motion_noise_r_t)
    , noise_r_r_(motion_noise_r_r)
  {
  }

public:
  // Update particle pose through the change in odometry data reading
  void sampleMotionModel(Particle& particle, const Stamped2DPose& pre_odom_pose,
                         const Stamped2DPose& cur_odom_pose)
  {
    // Increment
    double delta_x = cur_odom_pose.x_ - pre_odom_pose.x_;
    double delta_y = cur_odom_pose.y_ - pre_odom_pose.y_;
    double delta_theta = cur_odom_pose.theta_ - pre_odom_pose.theta_;
    normalizeTheta(delta_theta);

    // Increment added with noise
    delta_x +=
        sampleGaussian(noise_t_t_ * fabs(delta_x) + noise_x_y_ * fabs(delta_y) +
                       noise_t_r_ * fabs(delta_theta));
    delta_y +=
        sampleGaussian(noise_x_y_ * fabs(delta_x) + noise_t_t_ * fabs(delta_y) +
                       noise_t_r_ * fabs(delta_theta));
    delta_theta += sampleGaussian(
        noise_r_t_ * sqrt(delta_x * delta_x + delta_y * delta_y) +
        noise_r_r_ * fabs(delta_theta));
    normalizeTheta(delta_theta);

    particle.pre_pose_ = particle.cur_pose_;

    particle.cur_pose_.timestamp_ = cur_odom_pose.timestamp_;
    particle.cur_pose_.x_ += delta_x;
    particle.cur_pose_.y_ += delta_y;
    particle.cur_pose_.theta_ += delta_theta;
    normalizeTheta(particle.cur_pose_.theta_);
  }

  // Return random Gaussian noise with mean = 0
  double sampleGaussian(double covariance)
  {
    std::normal_distribution<double> distribution(0, covariance);
    return distribution(generator_);
  }
};

}  // namespace my_gmapping

#endif