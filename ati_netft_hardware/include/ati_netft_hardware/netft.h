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

#ifndef ATI_NETFT_HARDWARE__NETFT_H
#define ATI_NETFT_HARDWARE__NETFT_H

#include <composite_hardware_interface/composite_robot_hw.h>
#include </home/msdu/catkin_ws/src/hardware/netft_rdt_driver/include/netft_rdt_driver/netft_rdt_driver.h> //<netft_rdt_driver/netft_rdt_driver.h>
#include <geometry_msgs/WrenchStamped.h>


namespace ati_netft_hardware
{

/**
   @class netft_ethercat_hardware::NetFT
   @author Derek King
   @brief NetFT driver plugin for ethercat_hardware

   NetFT is analog converter for Force/Torque sensors from ATI IA.  
   http://www.ati-ia.com/products/ft/ft_NetFT.aspx

   This driver allows data published by NetFT over UDP to be available to 
   realtime controllers as analog inputs.  This driver also publishes data to ROS topic.

   @section ROS ROS interface

   @param address  IPV4 address for NetFT box. Example "192.168.1.1". 
*/

class NetFT : public composite_hardware_interface::DeviceHW
{
public:

  NetFT() :
      warn_force_lvl_(0), warn_torque_lvl_(0), error_force_lvl_(0), error_torque_lvl_(0), connected_(0), overload_(0)
  {
  }

  virtual bool configure(composite_hardware_interface::SharedInterfaceManager &ifaces,
                         const std::string &dev_name,
                         const ros::NodeHandle &dev_node);

  virtual bool start();

  virtual bool read(const ros::Time &time, const ros::Duration &period);

  virtual void stop();

private:

  static bool check(const geometry_msgs::Wrench& w, double max_force, double max_torque)
  {
    if (max_force)
      if (w.force.x * w.force.x + w.force.y * w.force.y + w.force.z * w.force.z > max_force * max_force)
        return false;

    if (max_torque)
      if (w.torque.x * w.torque.x + w.torque.y * w.torque.y + w.torque.z * w.torque.z > max_torque * max_torque)
        return false;

    return true;
  }

protected:
  ros::NodeHandle node_;
  
  std::string name_;
  std::string address_;
  std::string frame_id_;
  double warn_force_lvl_;
  double warn_torque_lvl_;
  double error_force_lvl_;
  double error_torque_lvl_;
  ros::Duration wait_period_;

  //! Driver interface to NetFT device.
  boost::shared_ptr<netft_rdt_driver::NetFTRDTDriver> netft_driver_;

  int connected_;
  int overload_;
  geometry_msgs::WrenchStamped wrench_;
};


}; //end namespace

#endif // ATI_NETFT_HARDWARE__NETFT_H
