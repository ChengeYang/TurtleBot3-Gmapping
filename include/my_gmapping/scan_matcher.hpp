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
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef SCAN_MATCHER_HPP
#define SCAN_MATCHER_HPP

namespace my_gmapping
{
class ScanMatcher
{
public:
  int max_iter_;
  double error_threshold_;

public:
  // Constructor
  ScanMatcher()
  {
  }

  ScanMatcher(int max_iter, double error_threshold)
    : max_iter_(max_iter), error_threshold_(error_threshold)
  {
  }

public:
  // Fit data to model by ICP, return data->model transform.
  bool icpMatching(const PointCloudXY& model, const PointCloudXY& data,
                   Stamped2DPose& transform)
  {
    // std::cerr << model.size_ << " " << data.size_ << std::endl;
    // Fail if input data has less than two points
    if (model.size_ < 2 or data.size_ < 2)
      return false;

    PointCloudXY model_matched_points;
    PointCloudXY data_matched_points;

    for (size_t iter = 0; iter < max_iter_; iter++)
    {

    }
  }
};

}  // namespace my_gmapping

#endif