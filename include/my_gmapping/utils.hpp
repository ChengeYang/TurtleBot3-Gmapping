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

#define INV_SQRT_2PI 0.3989422804014327

#include <math.h>

#include <eigen3/Eigen/Core>

#include <sensor_msgs/LaserScan.h>

namespace my_gmapping
{
// Class for representing 2D point [x, y]
class Point2D
{
public:
  double x_;
  double y_;

public:
  Point2D()
  {
  }

  Point2D(double x, double y) : x_(x), y_(y)
  {
  }
};

// Class for representing robot pose [x, y, theta]
class Pose2D
{
public:
  double x_;
  double y_;
  double theta_;  // [-pi, pi]

public:
  Pose2D()
  {
  }

  Pose2D(double x, double y, double theta) : x_(x), y_(y), theta_(theta)
  {
  }
};

// Class for representing robot pose [timestamp, x, y, theta]
class StampedPose2D
{
public:
  double timestamp_;
  double x_;
  double y_;
  double theta_;  // [-pi, pi]

public:
  StampedPose2D()
  {
  }

  StampedPose2D(double timestamp, double x, double y, double theta)
    : timestamp_(timestamp), x_(x), y_(y), theta_(theta)
  {
  }
};

// Class to store readings from laser scan input
class LaserScanReading
{
public:
  std::vector<double> ranges_;
  std::vector<double> bearings_;

public:
  LaserScanReading()
  {
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

// Transform robot coordinate [x, y] to translation vector t
Eigen::Vector3d xyToTranslationVector(double x, double y)
{
  Eigen::Vector3d t(x, y, 0.0);
  return t;
}

// Transform yaw angle theta to rotation matrix R
Eigen::Matrix3d thetaToRotationMatrix(double theta)
{
  Eigen::Matrix3d R;
  R << std::cos(theta), -std::sin(theta), 0, std::sin(theta), std::cos(theta),
      0, 0, 0, 1;
  return R;
}

// Transform robot pose by R & t
void transformPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                   StampedPose2D& pose)
{
  // Transfer pose into R & t
  Eigen::Vector3d t_pose = xyToTranslationVector(pose.x_, pose.y_);
  Eigen::Matrix3d R_pose = thetaToRotationMatrix(pose.theta_);

  R_pose = R_pose * R;
  t_pose = R_pose * t + t_pose;

  pose.x_ = t_pose[0];
  pose.y_ = t_pose[1];
  pose.theta_ = atan2(R_pose(1, 0), R_pose(0, 0));
}

// Transform robot pose by R & t
void transformPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                   const StampedPose2D& pose_init,
                   StampedPose2D& pose_transform)
{
  // Transfer initial pose into R & t
  Eigen::Vector3d t_pose = xyToTranslationVector(pose_init.x_, pose_init.y_);
  Eigen::Matrix3d R_pose = thetaToRotationMatrix(pose_init.theta_);

  R_pose = R_pose * R;
  t_pose = R_pose * t + t_pose;

  pose_transform.x_ = t_pose[0];
  pose_transform.y_ = t_pose[1];
  pose_transform.theta_ = atan2(R_pose(1, 0), R_pose(0, 0));
}

// Return probability of x from Gaussian PDF(mean, variance)
double computeGaussianLikelihood(double x, double mean, double variance)
{
  double temp = (x - mean) / variance;
  return INV_SQRT_2PI / variance * std::exp(-0.5 * temp * temp);
}

}  // namespace my_gmapping

#endif