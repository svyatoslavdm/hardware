/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012-2015, Bauman Moscow State Technical University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Bauman Moscow State Technical University,
 *      nor the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written
 *      permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/// \author Igor Kalevatykh <kalevatykhia@gmail.com>

#ifndef KUKA_HARDWARE__TARGET_H
#define KUKA_HARDWARE__TARGET_H

#include <algorithm>
#include "state.h"

namespace kuka_hardware
{

enum InterpolationModes
{
  INTERP_JOINT = 1, INTERP_LINEAR = 2,
};

struct RobotTarget
{
  enum { DOF = 6 };

  ros::Time time;
  int drive_power;
  int stop;
  int interpolation;
  int extrapolation;
  double internal_freq;
  double position[DOF];
  double velocity[DOF];
  double accelern[DOF];

  RobotTarget() :
      drive_power(0), stop(0), interpolation(INTERP_JOINT), extrapolation(0), internal_freq(10)
  {
    std::fill_n(position, DOF, 0.0);
    std::fill_n(velocity, DOF, 0.0);
    std::fill_n(accelern, DOF, 0.0);
  }

  void fillFromState(const RobotState& state)
  {
    std::copy(state.position, state.position + DOF, position);
    std::fill_n(velocity, DOF, 0.0);
    std::fill_n(accelern, DOF, 0.0);
  }

  inline bool operator !=(const RobotTarget& b) const
  {
    return !std::equal(position, position + DOF, b.position)
        || !std::equal(velocity, velocity + DOF, b.velocity)
        || !std::equal(accelern, accelern + DOF, b.accelern);
  }

  inline bool isEmpty() const { return time.isZero(); }
};

}

#endif /* KUKA_HARDWARE__TARGET_H */
