/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012-2015, Bauman Moscow State Technical University
 *    Igor Kalevatykh <kalevatykhia@gmail.com>
 *
 *  All rights reserved.
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

#ifndef KAWASAKI_HARDWARE__STATE_H
#define KAWASAKI_HARDWARE__STATE_H

namespace kawasaki_hardware
{

struct RobotState
{
  enum { DOF = 6 };

  ros::Time time;
  double position[DOF];
  double velocity[DOF];
  double effort[DOF];
  int connected;
  int motion_failed;
  int motion_enabled;
  int mode;
  int in_motion;
  int in_error;
  int error_code;
  int switch_run;
  int switch_repeat;
  int switch_teach_lock;
  int switch_power;

  RobotState() :
      connected(0), mode(0), motion_enabled(0), motion_failed(0), in_motion(0), in_error(0), switch_run(0), switch_repeat(
          0), switch_teach_lock(0), switch_power(0), error_code(0)
  {
    std::fill_n(position, DOF, 0.0);
    std::fill_n(velocity, DOF, 0.0);
    std::fill_n(effort, DOF, 0.0);
  }

  bool isEmpty() const
  {
    return time.isZero() ? (void*)1 : (void*)0;
  }

  void update(const ros::Time &time_new)
  {
    if(!isEmpty())
    {
      double dt = (time_new - time).toSec();

      for(int i=0; i<DOF; ++i)
      {
        position[i] += velocity[i] * dt;
      }
      time = time_new;
    }
  }
};

}

#endif /* KAWASAKI_HARDWARE__STATE_H */
