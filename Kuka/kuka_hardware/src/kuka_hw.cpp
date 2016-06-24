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
#include <string>
#include <vector>
#include <iostream> 
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>

#include <tinyxml.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <kuka_hardware/kuka_hw.h>
#include <kuka_hardware/internal/math.h>
#include <kuka_hardware/internal/udp_client.h>

using namespace std;
using namespace kuka_hardware;
using namespace hardware_interface;
using namespace composite_hardware_interface;

KukaHW::KukaHW() :
    work_(false), virtual_(false)
{
}

KukaHW::~KukaHW()
{
  stop();
}

bool KukaHW::configure(SharedInterfaceManager& ifaces, const string &name, const ros::NodeHandle& nh)
{
  name_ = name;
  node_ = nh;
  string ns = nh.getNamespace();

  nh.param("virtual", virtual_, false);

  if (!virtual_ && !nh.getParam("address", address_))
  {
    ROS_ERROR_NAMED(name, "No controller address (%s/address) found on parameter server.", ns.c_str());
    return false;
  }

  nh.param("port", port_, 49152);

  if (!nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_NAMED(name, "No joint names (%s/joints) found on parameter server.", ns.c_str());
    return false;
  }

  if (joint_names_.size() != DOF)
  {
    ROS_ERROR_NAMED(name, "Parameter list '%s/joint_names' should have %d names.", ns.c_str(), DOF);
    return false;
  }

  for (int i = 0; i < DOF; ++i)
  {
    JointStateHandle state_h(joint_names_[i], &state_.position[i], &state_.velocity[i], &state_.effort[i]);
    ifaces.registerHandle<JointStateInterface>(state_h);

    JointHandle pos_h(state_h, &target_.position[i]);
    ifaces.registerHandle<PositionJointInterface>(pos_h);
  }

  return true;
}

bool KukaHW::start()
{
  work_ = true;

  if (virtual_)
  {
    thread_ = boost::thread(boost::bind(&KukaHW::virtualLoop, this));
  }
  else
  {
    thread_ = boost::thread(boost::bind(&KukaHW::controlLoop, this));
  }

  return true;
}

bool KukaHW::read(const ros::Time &time, const ros::Duration &period)
{
  state_box_.get(state_);

  if ((time - state_.time) < ros::Duration(1.0))
  {
    state_.update(time);
    return true;
  }

  //state_.connected = 0;
  return false;
}

bool KukaHW::write(const ros::Time &time, const ros::Duration &period)
{
  if (target_.isEmpty())
  {
    if (state_.isEmpty())
    {
      return false;
    }
    target_.fillFromState(state_);
  }

  target_.time = time;
  target_box_.set(target_);

  return true;
}

void KukaHW::stop()
{
  if (work_)
  {
    work_ = false;
    thread_.join();
  }
}

void KukaHW::controlLoop()
{
  for (; work_; ros::Duration(1).sleep())
  {
    RobotState  state;
    RobotTarget target;

    try
    {
      udp::endpoint listen_end_point(boost::asio::ip::address::from_string(address_), port_);
      udp_client client(listen_end_point);

      vector<double> setpoint(DOF);
      vector<double> correction(DOF);

      while (work_ || state.connected)
      {
        boost::array<char, 1024> buffer;
        boost::asio::ip::udp::endpoint sender_endpoint;
        boost::system::error_code ec;
        client.receive(boost::asio::buffer(buffer), boost::posix_time::millisec(1000), sender_endpoint, ec);

        if (!ec)
        {
          string sender_address = sender_endpoint.address().to_string();

          static const string robot_address = sender_address;

          if(robot_address != sender_address)
          {
            throw runtime_error("Another robot try to connect:" + sender_address);
          }

          if (!state.connected)
          {
            ROS_WARN_NAMED(name_, "Robot connected: %s.", robot_address.c_str());
          }
          state.connected = 1;

          TiXmlDocument doc;
          if (!doc.Parse(buffer.data()) && doc.Error())
          {
            throw runtime_error("Can't parse robot response:\n" + string(buffer.data()));
          }

          TiXmlElement *root = doc.RootElement(),
              *aspos = root->FirstChildElement("ASPos"),
              *aipos = root->FirstChildElement("AIPos"),
              *macur = root->FirstChildElement("MACur"),
              *ipoc  = root->FirstChildElement("IPOC");

          if (!aspos || !aipos || !macur || !ipoc)
          {
            throw runtime_error("Can't parse robot response:\n" + string(buffer.data()));
          }

          double stamp = atol(ipoc->GetText());

          const string A[] = {"A1", "A2", "A3", "A4", "A5", "A6"};

          for (int i = 0; i < DOF; ++i)
          {
            int ret = TIXML_SUCCESS;

            double position_i;
            ret |= aipos->QueryDoubleAttribute(A[i], &position_i);
            double setpoint_i;
            ret |= aspos->QueryDoubleAttribute(A[i], &setpoint_i);
            double acurrent_i;
            ret |= macur->QueryDoubleAttribute(A[i], &acurrent_i);

            if (TIXML_SUCCESS != ret)
            {
              throw runtime_error("Can't parse robot response:\n" + string(buffer.data()));
            }

            setpoint[i] = math::deg2rad(setpoint_i);
            state.position[i] = math::deg2rad(position_i);
            state.effort[i] = acurrent_i;
          }

          state.time = ros::Time::now();
          state_box_.set(state);

          target_box_.get(target);
          if (!target.isEmpty() && !target.stop)
          {
            for (int i = 0; i < DOF; ++i)
            {
              correction[i] = target.position[i] - setpoint[i];
            }
          }

          boost::format response_format(
              "<Sen Type=\"ROS\">"
              "<EStr />"
              "<IPOC>%s</IPOC>"		      
              "<AKorr A1=\"%.4f\" A2=\"%.4f\" A3=\"%.4f\" A4=\"%.4f\" A5=\"%.4f\" A6=\"%.4f\"/>"
              "<Stop>%i</Stop>"
              "<Tech T21=\"1.09\" T22=\"2.08\" T23=\"3.07\" T24=\"4.06\" T25=\"5.05\" T26=\"6.04\" T27=\"7.03\" "
              "T28=\"8.02\" T29=\"9.01\" T210=\"10.00\"/>"
              "</Sen>");
   
	  
          int stop = work_ ? 0 : 1;

          boost::format& response = response_format % ipoc->GetText() % math::rad2deg(correction[0])
              % math::rad2deg(correction[1]) % math::rad2deg(correction[2]) % math::rad2deg(correction[3])
              % math::rad2deg(correction[4]) % math::rad2deg(correction[5]) % stop;

          std::string response_str = response.str();

          strcpy(buffer.c_array(), response_str.c_str());
          client.send(boost::asio::buffer(buffer), sender_endpoint);

          if (stop)
          {
            ROS_INFO_NAMED(name_, "Stopping robot...");
            ros::Duration(0.5).sleep();
            break;
          }
        }
        else
        {    
          if (state.connected)
          {
	    ROS_WARN_NAMED(name_, "EC: %s.", ec.message().c_str());
            ROS_WARN_NAMED(name_, "Robot disconnected.");
          }
          state.connected = 0;
          state_box_.set(state);
          //continue;
        }
      }
    }
    catch (exception& ex)
    {
      ROS_ERROR_NAMED(name_, "%s", ex.what());
    }

    if (state.connected)
    {
      ROS_WARN_NAMED(name_, "Robot disconnected.");
    }

    state.connected = 0;
    state_box_.set(state);
  }
}

void KukaHW::virtualLoop()
{
  RobotState state;
  RobotTarget target;

  ros::Rate rate(125);
  while (work_)
  {
    state.time = ros::Time::now();
    state.connected = 1;
    state_box_.set(state);

    target_box_.get(target);
    if (!target.isEmpty() && !target.stop)
    {
      for (int i = 0; i < DOF; ++i)
      {
        state.position[i] = target.position[i];
      }
    }

    rate.sleep();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kuka_hardware::KukaHW, composite_hardware_interface::DeviceHW)
