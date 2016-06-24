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

#ifndef KUKA_IK_SOLVER_H
#define KUKA_IK_SOLVER_H

#include <urdf/model.h>
#include <Eigen/Core>
#include <kdl/chainiksolver.hpp>
#include <kuka_kinematics/kuka_ik.h>
#include <tf_conversions/tf_kdl.h>

namespace kuka_kinematics
{

/** @class
 *  @brief ROS/KDL based interface for the inverse kinematics of the Kuka arm
 *
 *  This class provides a KDL based interface to the inverse kinematics of the Kuka arm. It inherits from the KDL::ChainIkSolverPos class
 *  but also exposes additional functionality to return multiple solutions from an inverse kinematics computation.
 */
class KukaIKSolver : public KDL::ChainIkSolverPos
{
public:

  enum
  {
    DOF = KukaIK::DOF
  };

  static const int SUCCESS = 1;
  static const int TIMED_OUT = -1;
  static const int NO_IK_SOLUTION = -2;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KukaIKSolver(const urdf::Model &robot_model, const std::string &root_frame_name, const std::string &tip_frame_name) :
    ChainIkSolverPos()
  {
    root_frame_name_ = root_frame_name;
    tip_frame_name_ = tip_frame_name;

    kuka_ik_.reset(new KukaIK(robot_model, root_frame_name, tip_frame_name));
  }

  virtual ~KukaIKSolver() {};

  std::string getFrameId() { return root_frame_name_; }
  bool isActive() { return !kuka_ik_ == false; }

  /**
   * Calculate inverse position kinematics, from cartesian
   *coordinates to joint coordinates.
   *
   * @param q_init initial guess of the joint coordinates
   * @param p_in position of the end-effector for which the IK is being solved
   * @param q_out a single inverse kinematic solution (if it exists).
   *
   * @return if < 0 something went wrong
   */
  virtual int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out)
  {
    if (kuka_ik_)
    {
      KukaIK::ArrayDofd result;
      if (kuka_ik_->computeIK6DOF(Eigen::Vector3d::Map(&p_in.p.data[0]), Eigen::Matrix3d::Map(&p_in.M.data[0]),
                                      KukaIK::ArrayDofd::Map(q_init.data.data()), result))
      {
        q_out.data = Eigen::VectorXd::Map(result.data(), KukaIK::DOF);
        return SUCCESS;
      }
    }

    return NO_IK_SOLUTION;
  }

  /**
   * Calculate inverse position kinematics, from cartesian
   *coordinates to joint coordinates.
   *
   * @param q_init initial guess of the joint coordinates
   * @param p_in position of the end-effector for which the IK is being solved
   * @param q_out all available inverse kinematic solutions.
   *
   * @return if < 0 something went wrong
   */
  int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out)
  {
    if (kuka_ik_)
    {
      std::list<KukaIK::ArrayDofd> solutions;
      if (kuka_ik_->computeIK6DOF(Eigen::Vector3d::Map(&p_in.p.data[0]), Eigen::Matrix3d::Map(&p_in.M.data[0]),
                                      KukaIK::ArrayDofd::Map(q_init.data.data()), solutions))
      {
        q_out.resize(solutions.size());
        std::list<KukaIK::ArrayDofd>::iterator solution = solutions.begin();
        for (size_t i = 0; i < solutions.size(); ++i, ++solution)
        {
          q_out[i].resize(KukaIK::DOF);
          q_out[i].data = Eigen::VectorXd::Map(solution->data(), KukaIK::DOF);
        }

        return SUCCESS;
      }
    }

    return NO_IK_SOLUTION;
  }

private:

  boost::shared_ptr<kuka_kinematics::KukaIK> kuka_ik_;
  std::string root_frame_name_;
  std::string tip_frame_name_;
};
}
#endif
