/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <boost/make_shared.hpp>
#include <ati_netft_hardware/netft.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <emergency_hardware_interface/emergency_state_interface.h>
#include <emergency_hardware_interface/emergency_command_interface.h>

namespace ati_netft_hardware
{

using namespace hardware_interface;
using namespace composite_hardware_interface;

bool NetFT::configure(SharedInterfaceManager &ifaces, const std::string &dev_name, const ros::NodeHandle &dev_node)
{
  if (!dev_node.getParam("address", address_))
  {
    ROS_ERROR("No '%s' controller address (%s/address) found on parameter server.", dev_name.c_str(),
              dev_node.getNamespace().c_str());
    return false;
  }

  if (!dev_node.getParam("frame_id", frame_id_))
  {
    ROS_ERROR("No '%s' controller frame_id (%s/frame_id) found on parameter server.", dev_name.c_str(),
              dev_node.getNamespace().c_str());
    return false;
  }

  dev_node.param("warning_level/force", warn_force_lvl_, 0.0);
  dev_node.param("warning_level/moment", warn_torque_lvl_, 0.0);
  dev_node.param("error_level/force", error_force_lvl_, 0.0);
  dev_node.param("error_level/torque", error_torque_lvl_, 0.0);

  wait_period_ = ros::Duration(0.5);

  ForceTorqueSensorHandle wrench_h(dev_name, frame_id_, &wrench_.wrench.force.x, &wrench_.wrench.torque.x);
  ifaces.registerHandle<ForceTorqueSensorInterface>(wrench_h);

  EmergencyStateHandle conn_h(dev_name + "/conn", dev_name, "Disconnected.", &connected_, 0);
  ifaces.registerHandle<EmergencyStateInterface>(conn_h);

  EmergencyStateHandle over_h(dev_name + "/overload", dev_name, "Overload.", &overload_, 2, 1);
  ifaces.registerHandle<EmergencyStateInterface>(over_h);

  name_ = dev_name;
  return true;
}

bool NetFT::start()
{
  try
  {
    netft_driver_ = boost::make_shared<netft_rdt_driver::NetFTRDTDriver>(address_);

    if (netft_driver_->waitForNewData())
    {
      netft_driver_->getData(wrench_);
      connected_ = 1;
      return true;
    }
  }
  catch (std::exception &e)
  {
    ROS_ERROR("[%s] Error constructing NetFT driver: %s", name_.c_str(), e.what());
  }

  connected_ = 0;
  return false;
}

bool NetFT::read(const ros::Time &time, const ros::Duration &period)
{
  netft_driver_->getData(wrench_);

  connected_ = time - wrench_.header.stamp < wait_period_;

  if(connected_)
  {
    overload_ =
        !check(wrench_.wrench, error_force_lvl_, error_torque_lvl_) ? 1 :
        !check(wrench_.wrench, warn_force_lvl_, warn_torque_lvl_) ? 2 : 0;
  }

  return true;
}

void NetFT::stop()
{
  netft_driver_.reset();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ati_netft_hardware::NetFT, composite_hardware_interface::DeviceHW)

