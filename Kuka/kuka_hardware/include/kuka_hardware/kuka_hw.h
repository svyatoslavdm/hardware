/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Bauman Moscow State Technical University
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

#ifndef KUKA_HARDWARE__KUKA_HW_H
#define KUKA_HARDWARE__KUKA_HW_H

#include <string>
#include <vector>
#include <boost/atomic.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <composite_hardware_interface/composite_robot_hw.h>

#include <kuka_hardware/internal/buffer.h>
#include <kuka_hardware/internal/state.h>
#include <kuka_hardware/internal/target.h>



using namespace composite_hardware_interface;

namespace kuka_hardware
{

class KukaHW : public DeviceHW
{
public:
  KukaHW();

  virtual ~KukaHW();

  virtual bool configure(SharedInterfaceManager& ifaces, const std::string &dev_name, const ros::NodeHandle& nh);

  virtual bool start();

  virtual bool read(const ros::Time &time, const ros::Duration &period);

  virtual bool write(const ros::Time &time, const ros::Duration &period);

  virtual void stop();

private:
  void controlLoop();

  void virtualLoop();

private:
  enum { DOF = 6 };

  ros::NodeHandle node_;

  std::string name_;
  std::string address_;
  int port_;
  bool virtual_;
  std::vector<std::string> joint_names_;

  RobotState state_;
  Buffer<RobotState> state_box_;

  RobotTarget target_;
  Buffer<RobotTarget> target_box_;

  boost::thread thread_;
  boost::atomic_bool work_;
};

}
#endif /* KUKA_HARDWARE__KUKA_HW_H */
