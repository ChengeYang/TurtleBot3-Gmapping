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

#include <eigen3/Eigen/Core>
#include <Eigen/Eigenvalues>

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
  // Update robot pose through the change in odometry data reading
  StampedPose2D updateMotionModel(const StampedPose2D& pose,
                                  const StampedPose2D& pre_odom_pose,
                                  const StampedPose2D& cur_odom_pose)
  {
    // Control
    double control_x = cur_odom_pose.x_ - pre_odom_pose.x_;
    double control_y = cur_odom_pose.y_ - pre_odom_pose.y_;
    double control_theta = cur_odom_pose.theta_ - pre_odom_pose.theta_;
    normalizeTheta(control_theta);

    StampedPose2D pose_new;
    pose_new.timestamp_ = cur_odom_pose.timestamp_;
    pose_new.x_ = pose.x_ + control_x;
    pose_new.y_ = pose.y_ + control_y;
    pose_new.theta_ = pose.theta_ + control_theta;
    normalizeTheta(pose_new.theta_);
    return pose_new;
  }

  // Get robot pose initial guess through the change in odometry data reading
  // Add Gaussian noise
  StampedPose2D sampleMotionModel(const StampedPose2D& pose,
                                  const StampedPose2D& pre_odom_pose,
                                  const StampedPose2D& cur_odom_pose)
  {
    // Control
    double control_x = cur_odom_pose.x_ - pre_odom_pose.x_;
    double control_y = cur_odom_pose.y_ - pre_odom_pose.y_;
    double control_theta = cur_odom_pose.theta_ - pre_odom_pose.theta_;
    normalizeTheta(control_theta);

    // Noise variance
    double var_x = noise_t_t_ * fabs(control_x) + noise_x_y_ * fabs(control_y) +
                   noise_t_r_ * fabs(control_theta);
    double var_y = noise_x_y_ * fabs(control_x) + noise_t_t_ * fabs(control_y) +
                   noise_t_r_ * fabs(control_theta);
    double var_theta =
        noise_r_t_ * sqrt(control_x * control_x + control_y * control_y) +
        noise_r_r_ * fabs(control_theta);

    // Control added with noise
    control_x += sampleGaussian(0, var_x);
    control_y += sampleGaussian(0, var_y);
    control_theta += sampleGaussian(0, var_theta);

    StampedPose2D pose_sample;
    pose_sample.timestamp_ = cur_odom_pose.timestamp_;
    pose_sample.x_ = pose.x_ + control_x;
    pose_sample.y_ = pose.y_ + control_y;
    pose_sample.theta_ = pose.theta_ + control_theta;
    normalizeTheta(pose_sample.theta_);
    return pose_sample;
  }

  // Motion model odometry: Probabilistic Robotics P134 Table5.5
  // Compute the likelihood P(Xt|Xt-1, Ut-1)
  double computePoseLikelihood(const StampedPose2D& pre_pose,
                               const StampedPose2D& cur_pose,
                               const StampedPose2D& pre_odom_pose,
                               const StampedPose2D& cur_odom_pose)
  {
    // Control
    double control_x = cur_odom_pose.x_ - pre_odom_pose.x_;
    double control_y = cur_odom_pose.y_ - pre_odom_pose.y_;
    double control_theta = cur_odom_pose.theta_ - pre_odom_pose.theta_;
    normalizeTheta(control_theta);

    // Difference between current and previous pose
    double delta_x = cur_pose.x_ - pre_pose.x_;
    double delta_y = cur_pose.y_ - pre_pose.y_;
    double delta_theta = cur_pose.theta_ - pre_pose.theta_;
    normalizeTheta(delta_theta);

    // Noise variance
    double var_x = noise_t_t_ * fabs(delta_x) + noise_x_y_ * fabs(delta_y) +
                   noise_t_r_ * fabs(delta_theta);
    double var_y = noise_x_y_ * fabs(delta_x) + noise_t_t_ * fabs(delta_y) +
                   noise_t_r_ * fabs(delta_theta);
    double var_theta =
        noise_r_t_ * sqrt(delta_x * delta_x + delta_y * delta_y) +
        noise_r_r_ * fabs(delta_theta);

    // Probability
    double p_x = computeGaussianLikelihood(control_x, delta_x, var_x);
    double p_y = computeGaussianLikelihood(control_y, delta_y, var_y);
    double p_theta =
        computeGaussianLikelihood(control_theta, delta_theta, var_theta);

    return p_x * p_y * p_theta;
  }

  // Generate random pose samples within given range
  std::vector<StampedPose2D>
  generateRandomPoseSamples(const StampedPose2D& pose, int num_samples,
                            double range_x, double range_y, double range_theta)
  {
    std::vector<StampedPose2D> poses;

    for (size_t i = 0; i < num_samples; i++)
    {
      StampedPose2D pose_random;
      pose_random.x_ = generateRandomNumber(pose.x_, range_x);
      pose_random.y_ = generateRandomNumber(pose.y_, range_y);
      pose_random.theta_ = generateRandomNumber(pose.theta_, range_theta);
      normalizeTheta(pose_random.theta_);
      poses.push_back(pose_random);
    }

    return poses;
  }

  // Sample new pose from multivariant Gaussian proposal
  StampedPose2D sampleGaussianProposal(const Eigen::Vector3d& mu,
                                       const Eigen::Matrix3d& sigma)
  {
    Eigen::Vector3d pose_eigen = sampleMultivariantGaussian(mu, sigma);

    StampedPose2D pose_new;
    pose_new.x_ = pose_eigen[0];
    pose_new.y_ = pose_eigen[1];
    pose_new.theta_ = pose_eigen[2];

    return pose_new;
  }

public:
  // Return random Gaussian noise with (mean, variance)
  double sampleGaussian(double mean, double variance)
  {
    std::normal_distribution<double> distribution(mean, variance);
    return distribution(generator_);
  }

  // Generate random number within [mean-range, mean+range]
  double generateRandomNumber(double mean, double range)
  {
    std::uniform_real_distribution<double> distribution(mean - range,
                                                        mean + range);
    return distribution(generator_);
  }

  // Sample from multivariant Gaussian distribution given mean and covaraince
  Eigen::Vector3d sampleMultivariantGaussian(const Eigen::Vector3d& mean,
                                             const Eigen::Matrix3d& covariance)
  {
    Eigen::Matrix3d transform;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance);
    transform = eigenSolver.eigenvectors() *
                eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();

    std::normal_distribution<double> distribution;

    return mean + transform * Eigen::Vector3d(distribution(generator_),
                                              distribution(generator_),
                                              distribution(generator_));
  }
};

}  // namespace my_gmapping

#endif