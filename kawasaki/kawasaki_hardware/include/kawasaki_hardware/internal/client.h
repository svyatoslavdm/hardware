/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Bauman Moscow State Technical University
 *    Igor Kalevatykh <kalevatykh@gmail.com>
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

#ifndef KAWASAKI_HARDWARE__CLIENT_H
#define KAWASAKI_HARDWARE__CLIENT_H

#include <algorithm>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

#include <kawasaki_hardware/internal/locale.h>
#include <kawasaki_hardware/internal/math.h>
#include <kawasaki_hardware/internal/state.h>
#include <kawasaki_hardware/internal/target.h>

namespace ip = boost::asio::ip;
namespace pt = boost::posix_time;

namespace kawasaki_hardware
{

class RobotClient
{
  enum { DOF = 6 };
  enum { PORT = 10050 };
  enum { VERSION = 1040 };

public:

  enum CommandStatus { SUCCESS };

  RobotClient(std::string address) :
      address_(address), time_(0), ping_(0)
  {
    connect();
  }

  virtual ~RobotClient()
  {
    disconnect();
  }

  operator void*() const
  {
    return state_.connected && stream_.good() ? (void*)1 : (void*)0;
  }

  void connect()
  {
    disconnect();

    base_time_ = ros::Time::now();

    std::string port = boost::lexical_cast<std::string>(PORT);
    stream_.connect(address_, port);

    ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
    for (; !stream_ && ros::Time::now() < deadline; ros::Duration(0.1).sleep())
    {
        stream_.clear();
        stream_.connect(address_, port);
    }

    if (!stream_)
      throw std::runtime_error("Cannot connect to the Kawasaki controller (" + address_ + ").");

    ip::tcp::no_delay option(true);
    stream_.rdbuf()->set_option(option);
    stream_.imbue(locale_);

    for (int i = 0; i < 10; ++i)
      ping();

    state_.connected = true;
  }

  void disconnect()
  {
    if (state_.connected && stream_.good())
    {
      send(SHUTDOWN);
      std::flush(stream_);
      stream_.close();
    }

    state_.connected = false;
  }

  inline const RobotState& state()
  {
    return state_;
  }

  inline bool ping()
  {
    return send(NOP);
  }

  inline bool setSpeedLimit(double percent)
  {
    std::vector<double> args;
    args.push_back(100.0 * percent);

    return send(SPEED_LIMIT, args);
  }

  inline bool setMaxVelocities(const std::vector<double>& velocities)
  {
    return send(SET_MAX_VELOCITIES, velocities);
  }

  bool enableMotion(bool enable = true)
  {
    if(!state_.motion_enabled == !enable)
    {
      state_.motion_failed = 0;
      return true;
    }

    if (send(enable ? MOTION_ENABLE : MOTION_DISABLE))
    {
      ros::Time deadline = ros::Time::now() + ros::Duration(2.0);
      while (ros::Time::now() < deadline)
      {
        if(!state_.motion_enabled == !enable)
        {
          state_.motion_failed = 0;
          return true;
        }

        ping();
      }
    }

    state_.motion_failed = 1;
    return false;
  }

  inline bool disableMotion()
  {
    return enableMotion(false);
  }

  inline bool move(const RobotTarget& cmd)
  {
    std::vector<double> args;
    args.push_back(toControllerTime(cmd.time));
    args.push_back(cmd.extrapolation ? 3 : 0);
    args.push_back(cmd.internal_freq);
    std::transform(cmd.position, cmd.position + DOF, std::back_inserter(args), &math::rad2deg);
    std::transform(cmd.velocity, cmd.velocity + DOF, std::back_inserter(args), &math::rad2deg);
    std::transform(cmd.accelern, cmd.accelern + DOF, std::back_inserter(args), &math::rad2deg);

    if(cmd.interpolation == INTERP_JOINT || fabs(cmd.position[4]) < math::deg2rad(2))
    {
      return send(JMOVE, args);
    }
    else
    {
      return send(LMOVE, args);
    }
  }

  inline bool stop()
  {
    return send(STOP);
  }

private:

  bool send(int cmd, const std::vector<double>& args = std::vector<double>())
  {
    stream_.expires_from_now(pt::millisec(cmd == MOTION_ENABLE ? 10000 : 1000));
    ros::Duration start = ros::Time::now() - base_time_;
    
    // send request
    std::ostringstream pack;
    pack.imbue(locale_);
    std::ostream_iterator<double> ipack(pack, " ");
    ipack++ = VERSION;
    ipack++ = cmd;
    ipack++ = args.size();
    pack << std::fixed << std::setprecision(4);
    std::copy(args.begin(), args.end(), ipack);
    std::string request = pack.str();

    stream_ << request;

    //std::cout << "request: " << request << std::endl;


    // receive response
    int version;
    stream_ >> version;

    //std::cout << "response version: " << version << std::endl;

    boost::system::error_code code;
    code = stream_.error();
    
    if (code.value() != 0)
    {
	ROS_INFO("COMMAND: %d", cmd);
	ROS_INFO("ERROR CODE: %d", code.value());
    }
    if (!stream_.good())
    {
      throw std::runtime_error("Connection aborted due to an error.");
    }

    if (version != VERSION)
    {
      throw std::runtime_error("Controller firmware is not supported.");
    }

    stream_ >> state_.error_code;
    stream_ >> state_.switch_run;
    stream_ >> state_.switch_repeat;
    stream_ >> state_.switch_teach_lock;
    stream_ >> state_.switch_power;
    stream_ >> state_.motion_enabled;

    state_.in_error = state_.error_code != 0;
    state_.mode = !state_.switch_run || state_.switch_teach_lock ? 1 : 2;
    state_.in_motion = 0;

    std::istream_iterator<double> it(stream_);
    double time = *it;
    state_.time = toRosTime(time);

    for (int i = 0; i < DOF; ++i)
    {
      state_.position[i] = math::deg2rad(*++it);
      //state_.velocity[i] = diff_[i](time, state_.position[i]);

      state_.in_motion |= fabs(state_.velocity[i]) > 1e-2;
    }

    ros::Duration end = ros::Time::now() - base_time_;
    stream_.expires_from_now(pt::pos_infin);

    time_ = math::expSmooth(time_, time - 0.5 * (start + end).toSec(), 0.01);
    ping_ = math::expSmooth(ping_, 0.5 * (end - start).toSec(), 0.01);

    return state_.in_error == 0;
  }

  inline double toControllerTime(const ros::Time& time)
  {
    return (time - base_time_).toSec() + time_;
  }

  inline ros::Time toRosTime(double time)
  {
    return base_time_ + ros::Duration(time - time_);
  }

private:

  enum CommandCode
  {
    NOP, MOTION_ENABLE, MOTION_DISABLE, SPEED_LIMIT, SET_MAX_VELOCITIES, STOP, JMOVE, LMOVE, SHUTDOWN = 255
  };

  std::string address_;
  controller_locale locale_;
  ip::tcp::iostream stream_;

  ros::Time base_time_;
  double time_;
  double ping_;

  RobotState state_;
  //math::Diff diff_[DOF];
};

}

#endif /* KAWASAKI_HARDWARE__CLIENT_H */
