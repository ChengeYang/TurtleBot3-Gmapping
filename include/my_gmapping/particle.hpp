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
 * Particle class to maintain all information about a single particle.
 *
 * Author: Chenge Yang
 * Email: chengeyang2019@u.northwestern.edu
 ****************************************************************************/

#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <iostream>

#include <ros/ros.h>

#include "my_gmapping/mapper.hpp"
#include "my_gmapping/utils.hpp"

namespace my_gmapping
{
class Particle
{
public:
  // Current and previous robot pose: [x, y, theta]
  Stamped2DPose cur_pose_;
  Stamped2DPose pre_pose_;  // TODO: pre_pose_ can be discarded

  // Map
  Mapper mapper_;

  // Weight
  double weight_;

public:
  // Constructor
  Particle()
  {
  }

  Particle(Stamped2DPose init_pose, Mapper mapper, double weight)
    : cur_pose_(init_pose)
    , pre_pose_(init_pose)
    , mapper_(mapper)
    , weight_(weight)
  {
  }
};

}  // namespace my_gmapping

#endif