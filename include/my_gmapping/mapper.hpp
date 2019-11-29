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
 * Map class for maintaining and updating 2D Grid map.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef Mapper_HPP
#define Mapper_HPP

#include <iostream>

#include <ros/ros.h>

#include <vector>

namespace my_gmapping
{
class Mapper
{
public:
  // Input map parameters
  double resolution_;
  double x_max_;
  double x_min_;
  double y_max_;
  double y_min_;

  // Map always centered at (0, 0)
  int x_size_;
  int y_size_;
  std::vector<double> map_;

public:
  // Constructor
  Mapper()
  {
  }

  Mapper(double resolution, double x_max, double x_min, double y_max,
         double y_min)
    : resolution_(resolution)
    , x_max_(x_max)
    , x_min_(x_min)
    , y_max_(y_max)
    , y_min_(y_min)
    , x_size_(std::ceil((x_max - x_min) / resolution_))
    , y_size_(std::ceil((y_max - y_min) / resolution_))
    , map_(x_size_ * y_size_, 0)
  {
  }

public:
  std::vector<double> getMap()
  {
    return map_;
  }

  // Convert [x, y] position to the index of map vector
  int posToIndex(double x, double y)
  {
    int x_index = std::floor((x - x_min_) / resolution_);
    int y_index = std::floor((y - y_min_) / resolution_);
    return x_index * y_size_ + y_index;
  }
};

}  // namespace my_gmapping

#endif