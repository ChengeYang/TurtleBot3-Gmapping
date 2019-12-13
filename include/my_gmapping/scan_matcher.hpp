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
 * Iterative Closest Point (ICP) based 2D scan matching algorithm.
 * Implemented using Eigen3 Library.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef SCAN_MATCHER_HPP
#define SCAN_MATCHER_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <pcl/kdtree/kdtree_flann.h>

#include "my_gmapping/utils.hpp"

namespace my_gmapping
{
class ScanMatcher
{
public:
  int max_iter_;         // Max number of iteration
  int max_matched_pts_;  // Max points in matching
  double epsilon_;       // Terminate if two iterations changes less than it
  double termination_error_;  // Terminate if squared distance is less than it
  double matching_factor_;    // Factor to filter out matching points

public:
  // Constructor
  ScanMatcher()
  {
  }

  ScanMatcher(int max_iter, int max_matched_pts, double epsilon,
              double termination_error, double matching_factor)
    : max_iter_(max_iter)
    , max_matched_pts_(max_matched_pts)
    , epsilon_(epsilon)
    , termination_error_(termination_error)
    , matching_factor_(matching_factor)
  {
  }

public:
  // Find data->model transformation by ICP (R & t)
  // Apply transform to initial pose and return transformed pose
  bool icpMatching(const std::vector<Eigen::Vector3d>& model,
                   const std::vector<Eigen::Vector3d>& data,
                   const StampedPose2D& pose_init,
                   StampedPose2D& pose_scan_match)
  {
    // Fail if map is ready or input data has less than two points
    if (model.size() < max_matched_pts_ or data.size() < 2)
    {
      std::cerr << "Scan match failed. No enough points." << std::endl;
      return false;
    }

    // Determine step length for choosing matching points
    int step = 1;
    if (data.size() > max_matched_pts_)
      step = std::ceil(double(data.size()) / double(max_matched_pts_));

    // Container for matched points
    std::vector<Eigen::Vector3d> matched_points_model;
    std::vector<Eigen::Vector3d> matched_points_data;

    // Iteration variables
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t(0.0, 0.0, 0.0);
    double matching_threshold = std::numeric_limits<double>::max();
    double pre_average_squared_dist = std::numeric_limits<double>::max();
    double cur_average_squared_dist = 0.0;

    // Construct K-D Tree
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_model(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < model.size(); i++)
    {
      pcl::PointXYZ point(model[i][0], model[i][1], model[i][2]);
      pcl_model->push_back(point);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
    kd_tree.setInputCloud(pcl_model);

    for (size_t iter = 0; iter < max_iter_; iter++)
    {
      // Initialization
      matched_points_model.clear();
      matched_points_data.clear();
      double squared_distance_sum = 0.0;

      // Compute matched points by K-D Tree
      for (size_t i_data = 0; i_data < data.size(); i_data += step)
      {
        // Transform point in data by R & t
        Eigen::Vector3d point_data = R * data[i_data] + t;

        // Search by K-D Tree
        std::vector<int> index(1);
        std::vector<float> squared_distance(1);
        pcl::PointXYZ point_pcl(point_data[0], point_data[1], point_data[2]);
        if (!kd_tree.nearestKSearch(point_pcl, 1, index, squared_distance))
          return false;

        // Record matched points
        if (squared_distance[0] < matching_threshold)
        {
          matched_points_model.push_back(model[index[0]]);
          matched_points_data.push_back(data[i_data]);
          squared_distance_sum += squared_distance[0];
        }
      }

      // Compute center of mass for matched points
      Eigen::Vector3d center_model = computeCenterOfMass(matched_points_model);
      Eigen::Vector3d center_data = computeCenterOfMass(matched_points_data);

      // Compute W
      Eigen::Matrix3d W;
      for (size_t i = 0; i < matched_points_data.size(); i++)
      {
        W += (matched_points_model[i] - center_model) *
             ((matched_points_data[i] - center_data).transpose());
      }

      // Compute R and t by SVD
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU |
                                                   Eigen::ComputeFullV);
      R = svd.matrixU() * (svd.matrixV().transpose());
      t = center_model - R * center_data;

      // Check for convergence
      cur_average_squared_dist =
          squared_distance_sum / matched_points_data.size();
      double difference = pre_average_squared_dist - cur_average_squared_dist;
      if (cur_average_squared_dist < termination_error_ or
          difference < epsilon_)
      {
        // Convert R & t to robot pose
        transformPose(R, t, pose_init, pose_scan_match);
        return true;
      }

      pre_average_squared_dist = cur_average_squared_dist;
      matching_threshold = cur_average_squared_dist * matching_factor_;
    }  // End of iteration

    std::cerr << "Scan match Failed. Not converging." << std::endl;
    return false;
  }

  // Compute center of mass for a group of points
  Eigen::Vector3d
  computeCenterOfMass(const std::vector<Eigen::Vector3d>& points)
  {
    Eigen::Vector3d center(0.0, 0.0, 0.0);

    for (size_t i = 0; i < points.size(); i++)
      center += points[i];

    return center / points.size();
  }
};

}  // namespace my_gmapping

#endif