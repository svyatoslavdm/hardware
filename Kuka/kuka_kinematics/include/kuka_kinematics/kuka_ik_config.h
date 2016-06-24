/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012-2015, Igor Kalevatykh, Svyatoslav Moroshkin, Bauman Moscow State Technical University
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

/// \author Svyatoslav Moroshkin <svyatoslavdm@gmail.com>  inspired by Igor Kalevatykh <kalevatykhia@gmail.com>

#ifndef KUKA_IK_CONFIGURATION_H_
#define KUKA_IK_CONFIGURATION_H_

#include <math.h>

namespace kuka_kinematics
{

/**
 * @brief Manipulator configuration indexes.
 */
struct Configuration
{
  enum Range
  {
    Count = 8
  };

  enum ShoulderIndex
  {
    Righty = 1, Lefty = -1
  } shoulder;

  enum ElbowIndex
  {
    Above = 1, Below = -1
  } elbow;

  enum WristIndex
  {
    Up = 1, Down = -1
  } wrist;

  Configuration() :
      shoulder(Righty), elbow(Above), wrist(Up)
  {
  }

  Configuration(int index) :
      shoulder(index & 0x01 ? Lefty : Righty), elbow(index & 0x02 ? Below : Above), wrist(index & 0x04 ? Down : Up)
  {
  }

  template<typename T>
    Configuration(T s_index, T e_index, T w_index) :
        shoulder(std::signbit(s_index) ? Lefty : Righty), elbow(std::signbit(e_index) ? Below : Above),
        wrist(std::signbit(w_index) ? Down : Up)
    {
    }

  operator int()
  {
    return (shoulder == Lefty ? 0x01 : 0x00) | (elbow == Below ? 0x02 : 0x00) | (wrist == Down ? 0x04 : 0x00);
  }

  bool operator ==(const Configuration& other) const
  {
    return shoulder == other.shoulder && elbow == other.elbow && wrist == other.wrist;
  }

  bool operator !=(const Configuration& other) const
  {
    return shoulder != other.shoulder || elbow != other.elbow || wrist != other.wrist;
  }

  std::string toString() const
  {
    std::stringstream os;
    if(shoulder == Righty) os << "Righty,"; else os << "Lefty,";
    if(elbow == Above) os << "Above,"; else os << "Below,";
    if(wrist == Up) os << "Up"; else os << "Down";
    return os.str();
  }
};

}
#endif
