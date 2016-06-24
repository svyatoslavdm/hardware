/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Igor Kalevatykh, Bauman Moscow State Technical University
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

#include <string>
#include <vector>

#include <kawasaki_hardware/kawasaki_hw.h>
#include <kawasaki_hardware/internal/client.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <emergency_hardware_interface/emergency_command_interface.h>

using namespace kawasaki_hardware;
using namespace hardware_interface;
using namespace composite_hardware_interface;
using namespace joint_limits_interface;

KawasakiHW::KawasakiHW() :
    speed_limit_(0.0), standby_after_(0.0), work_(false)
{ }

KawasakiHW::~KawasakiHW()
{
  stop();
}

bool KawasakiHW::configure(SharedInterfaceManager& ifaces, const std::string &name, const ros::NodeHandle& nh)
{
  name_ = name;
  node_ = nh;

  if (!nh.getParam("address", address_))
  {
    ROS_ERROR("[%s] No controller address (%s/address) found on parameter server.", name.c_str(),
              nh.getNamespace().c_str());
    return false;
  }

  if (!nh.getParam("joints", joint_names_))
  {
    ROS_ERROR("[%s] No joint names (%s/joints) found on parameter server.", name.c_str(),
              nh.getNamespace().c_str());
    return false;
  }

  if (joint_names_.size() != DOF)
  {
    ROS_ERROR("[%s] Parameter '%s/joint_names' should have %d names.", name.c_str(), nh.getNamespace().c_str(), DOF);
    return false;
  }

  max_velocities_.resize(DOF);
  for (int i = 0; i < joint_names_.size(); ++i)
  {
    if (!nh.getParam(joint_names_[i] + "/max_velocity", max_velocities_[i]))
    {
      ROS_ERROR("[%s] No joint max velocity (%s/%s/max_velocity) found on parameter server.", name.c_str(),
                nh.getNamespace().c_str(), joint_names_[i].c_str());
      return false;
    }
  }

  nh.getParam("speed_limit", speed_limit_);
  nh.getParam("standby_after", standby_after_);
  nh.getParam("extrapolation", target_.extrapolation);
  nh.getParam("internal_freq", target_.internal_freq);

  for (int i = 0; i < DOF; ++i)
  {
    JointStateHandle state_h(joint_names_[i], &state_.position[i], &state_.velocity[i], &state_.effort[i]);
    ifaces.registerHandle<JointStateInterface>(state_h);

    JointHandle pos_h(state_h, &target_.position[i]);
    ifaces.registerHandle<PositionJointInterface>(pos_h);

    PosVelAccJointHandle pva_h(state_h, &target_.position[i], &target_.velocity[i], &target_.accelern[i]);
    ifaces.registerHandle<PosVelAccJointInterface>(pva_h);
  }

  std::vector<EmergencyStateHandle> esh;
  esh.push_back(EmergencyStateHandle(name + "/disconnected", name + " disconnected.", &state_.connected, 0));
  esh.push_back(EmergencyStateHandle(name + "/motion", name + " can't start motion.", &state_.motion_failed));
  esh.push_back(EmergencyStateHandle(name + "/error", name + " Error on the robot side.", &state_.error_code));

  for (int i = 0; i < esh.size(); ++i)
  {
    ifaces.registerHandle<EmergencyStateInterface>(esh[i]);
  }

  EmergencyHandle estop_h(name, esh, &target_.stop);
  ifaces.registerHandle<EmergencyInterface>(estop_h);

  pub = node_.advertise<std_msgs::Float32>( "/target_joint", 1 );
  
  return true;
}

bool KawasakiHW::start()
{
  work_ = true;
  thread_ = boost::thread(boost::bind(&KawasakiHW::mainLoop, this));

  ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
  for (; ros::Time::now() < deadline; ros::Duration(0.1).sleep())
  {
    state_box_.get(state_);

    if (!state_.isEmpty())
    {
      target_.fillFromState(state_);
      return true;
    }
  }

  stop();
  return false;
}

bool KawasakiHW::read(const ros::Time &time, const ros::Duration &period)
{
  state_box_.get(state_);

  if((time - state_.time) < ros::Duration(1.0))
  {
    state_.update(time);
    return true;
  }

  //state_.disconnected = 1;
  return false;
}

bool KawasakiHW::write(const ros::Time &time, const ros::Duration &period)
{
  target_.time = time;
  target_box_.set(target_);
  return true;
}

void KawasakiHW::stop()
{
  if(work_)
  {
    work_ = false;
    thread_.join();
  }
}

void KawasakiHW::mainLoop()
{
  for (; work_; ros::Duration(1).sleep())
  {
    RobotState state;
    RobotTarget target, last_target;
    bool standby = standby_after_ > 0;

    try
    {
      RobotClient manipulator(address_);
      ROS_INFO("Robot '%s' connected.", name_.c_str());
      state.connected = 1;

      if (!manipulator.setMaxVelocities(max_velocities_))
      {
        throw std::runtime_error("Cannot set maximum velocities.");
      }

      if (speed_limit_ && !manipulator.setSpeedLimit(speed_limit_))
      {
        throw std::runtime_error("Cannot set speed limit.");
      }

      ros::WallRate rate(10);
      while (work_.load(boost::memory_order_acquire))
      {
        state = manipulator.state();
        state_box_.set(state);
        rate.sleep();

        target_box_.get(target);

        if (!target.isEmpty())
        {
          if (target.stop)
          {
            manipulator.stop();
            continue;
          }

          if (standby_after_)
          {
            if (target != last_target)
            {
              if (!last_target.isEmpty()) 
	      { 
		standby = false; 
	      }
              last_target = target;
            }
            else if (standby_after_ < (ros::Time::now() - last_target.time).toSec())
            {
              standby = true;
              manipulator.disableMotion();
            }
          }

          if (!standby)
          {
            if (manipulator.enableMotion())
            {
	      if (name_ == "left_arm")
	      {
// 		ROS_INFO_STREAM("Target for 6th joint: " << target.position[5]);  
		std_msgs::Float32 msg;
		msg.data = target.position[5];
		pub.publish(msg);
	      }
              manipulator.move(target);
            }
            else
            {
              ROS_WARN_THROTTLE(1, "[%s] Could not execute motion program. Check motor power is on, etc.", name_.c_str());
            }
            continue;
          }
        }

        manipulator.ping();
      }
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("[%s] %s", name_.c_str(), ex.what());
    }

    state.connected = 0;
    state_box_.set(state);

    ROS_WARN("[%s] Robot disconnected.", name_.c_str());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kawasaki_hardware::KawasakiHW, composite_hardware_interface::DeviceHW)
