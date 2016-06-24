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

#ifndef KUKA_IK_H
#define KUKA_IK_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <urdf/model.h>
#include <kuka_kinematics/kuka_ik_config.h>
namespace kuka_kinematics
{

using namespace Eigen;

/** @class
 *  @brief Inverse kinematics for the Kuka robotic arm.
 */

class KukaIK
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Manipulator degrees of freedom number.
   */
  enum { DOF = 6 };

  /// Array of joint positions.
  typedef Eigen::Array<double, DOF, 1> ArrayDofd;

  /// Array of joint positions.
  typedef Eigen::Array<double, DOF, Configuration::Count> MatrixDofd;

  /// Get joints names.
  const std::vector<std::string>& jointNames() const { return jointNames_; }

  /// Get joints lower limits.
  const ArrayDofd& lowerLimit() const { return lowerLimit_; }
  void lowerLimit(const ArrayDofd& limit) { lowerLimit_ = limit; }

  /// Get joints upper limits.
  const ArrayDofd& upperLimit() const { return upperLimit_; }
  void upperLimit(const ArrayDofd& limit) { upperLimit_ = limit; }

  /// Set joints weight for searching nearest solution.
  void jointWeight(const ArrayDofd& weight) { jointWeight_ = weight; }
 /**
   * @brief Construct the solver by providing a urdf::Model and a root and tip name.
   * @param robotModel - a urdf::Model representation of the Kuka robot model
   * @param rootName - the root frame name of the arm
   * @param tipName - the tip frame name of the arm
   */
  KukaIK(const urdf::Model &robotModel, const std::string &rootName, const std::string &tipName) :
    jointNames_(DOF)
  {
    boost::shared_ptr<const urdf::Link> link = robotModel.getLink(tipName);    
    for(int i=DOF-1; i>=0; i--)
    {
      boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
      if(!link || !joint)
        throw std::runtime_error("IK: Chain from '" + rootName + "' to '" + tipName + "' does not have 6 joints.");

      const double one = 1 - 1e-5;
      const urdf::Pose& offset = joint->parent_to_joint_origin_transform;

      Vector3d length(Vector3d::Map(&offset.position.x));

      if (!joint->safety && !joint->limits && joint->type != urdf::Joint::FIXED)
        throw std::runtime_error("IK: Joint '" + joint->name + "' does not have limits.");
      
      if (joint->type != urdf::Joint::FIXED)
      {
	jointNames_[i] = joint->name;
	length_    [i] = length.norm();
	lowerLimit_[i] = joint->limits->lower + 1e-5;
	upperLimit_[i] = joint->limits->upper - 1e-5;
      }
      else 
      {
	if (link->getParent()->name == "kuka/link_6")
	{
	  const urdf::Pose& offset_ = joint->parent_to_joint_origin_transform;
	  Vector3d pos(offset_.position.x, offset_.position.y, offset_.position.z);
	  Quaterniond rot(offset_.rotation.w, offset_.rotation.x, offset_.rotation.y, offset_.rotation.z);
	  
	  L_6_T_T = pos;
	  tau_6_T = rot.toRotationMatrix();	 
	}
	i++;
      }
      link = robotModel.getLink(link->getParent()->name);
    }
    baseRot.setIdentity();
    basePos.setZero();

    length_[0] = length_[0] + 0.183;
    length_[1] = 0.025;
    
    while(link && link->name != rootName)
    {
      boost::shared_ptr<urdf::Joint> joint = link->parent_joint;

      if (!joint || joint->type != urdf::Joint::FIXED)
        throw std::runtime_error("IK: Link '" + link->name + "' should have fixed parent joint.");
   
      const urdf::Pose& offset = joint->parent_to_joint_origin_transform;
      Vector3d pos(offset.position.x, offset.position.y, offset.position.z);
      Quaterniond rot(offset.rotation.w, offset.rotation.x, offset.rotation.y, offset.rotation.z);

      basePos = rot.toRotationMatrix().transpose() * basePos + pos;
      baseRot = rot.toRotationMatrix().transpose() * baseRot;

      link = robotModel.getLink(link->getParent()->name);
    }
    jointWeight_ = ArrayDofd::LinSpaced(6, 1);
  }
  /**
   * @brief Return configuration index for the specific joint pose.
   * @param q - joint state.
   * @param singDist - approx. distance to singularity in radians.
   */
  Configuration getConfig(const ArrayDofd &q, /*out*/ double* singDist = 0) const
  {
    Vector3d l01(l1, 0, 0);
    Vector3d l12(l2 * cos(-q[1]), l2 * sin(-q[1]), 0);  
    Vector3d l23(l3 * cos(q[2] + M_PI / 2 - (-q[1])), l3 * sin(q[2] + M_PI / 2 - (-q[1])), 0);  
    Vector3d l34(l4 * cos(-q[1] + q[2]), l4 * sin(-q[1] + q[2]), 0);  
    Vector3d l24(l23 + l34);
    Vector3d l04(l01 + l12 + l23 + l34);

    double s = atan2(l04.x(), l04.y());
    double e = atan2(l24.cross(l12).z(), l24.dot(l12));
    double w = -q[4];

    if (singDist)
      *singDist = Array3d(s, e, w).abs().minCoeff();
    return Configuration(s, e, w);
  }
  /**
   * @brief Return configuration index for the specific joint pose.
   * @param q - joint state.
   * @param singDist - approx. distance to singularity in radians.
   */
  Configuration getConfig(const std::vector<double> &q, /*out*/ double* singDist = 0) const
  {
    assert(q.size() == DOF);
    return getConfig(ArrayDofd::Map(q.data()), singDist);
  }

  /**
   * @brief Compute IK solution based on kinematic indexes.
   * @param tipPos - end-effector postion
   * @param tipRot - end-effector orientation
   * @param indexes - kinematic indexes
   * @param qIn - initial joint state
   * @param qOut - IK solution
   * @return true if IK solved, false otherwise.
   */
  bool computeIK6DOF(const Vector3d &tipPos, const Matrix3d &tipRot, const Configuration& ind,
                     const ArrayDofd &qIn, /*out*/ ArrayDofd &qOut) const
  {
/*    std::cout << "---------- IK IN -----------------\n";  */  
    const double eps = std::numeric_limits<double>::epsilon();

    Vector3d x0(baseRot.row(0));
    Vector3d y0(baseRot.row(1));
    Vector3d z0(baseRot.row(2));
    
    double l_tool = L_6_T_T.norm();
    
    Vector3d x_t(tipRot.row(0));
    Vector3d y_t(tipRot.row(1));
    Vector3d z_t(tipRot.row(2));
        
    Vector3d l04 = tipPos - basePos - z0 * l0 - z_t * (l5 + l_tool);

    Vector3d z1 = l04.cross(z0) * ind.shoulder;

    if( z1.norm() < eps)
    {
      z1 = - x0 * sin(qIn(0)) - y0 * cos(qIn(0));
      qOut(0) = qIn(0);
    }
    else
    {
      z1.normalize();
      qOut(0) = atan2(-z1.dot(x0), 
		      z1.dot(-y0));
    }

    Vector3d y1 = z0.cross(z1);
    Vector3d l14 = l04 - y1 * l1;

    double a = 0.5 + (l2 * l2 - l3 * l3 - l4 * l4) / (2 * l14.squaredNorm());
    double n2 = l2 * l2 - a * a * l14.squaredNorm();

    if (n2 < eps)
    {
      return false;
    }

    Vector3d n = z1.cross(l14.normalized()) * sqrt(n2) * ind.elbow;

    Vector3d l12 = a * l14 + n;
    Vector3d l24 = l14 - l12;
    Vector3d l34 = l24 * l4 + l24.cross(z1) * l3;
    Vector3d z3 = l34.normalized();

    Vector3d z4 = z3.cross(z_t) * ind.wrist;

    if (z4.norm() < eps)
    {
      z4 = - x_t * cos(qIn(5)) - y_t * sin(qIn(5));
      qOut(5) = qIn(5);
    }
    else
    {
      z4.normalize();
      qOut(5) = atan2(z4.cross(-x_t).dot(-z_t), 
		      z4.dot(-x_t));
    }

    qOut(1) = atan2(l12.cross(y1).dot(z1), 
		    l12.dot(y1));
    qOut(2) = atan2(z3.cross(l12).dot(z1), 
		    z3.dot(l12));
    qOut(3) = atan2(z4.cross(z1).dot(z3), 
		    z4.dot(z1));
    qOut(4) = atan2(z_t.cross(z3).dot(z4), 
		    z_t.dot(z3));

    bool joint_limits = checkJointLimits(qIn, qOut);
    
    return joint_limits;
  }

  /**
   * @brief Compute all available IK solutions and sort them based on distance from initial state.
   * @param tipPos - end-effector postion
   * @param tipRot - end-effector orientation
   * @param qIn - initial joint state
   * @param solution - IK solution list
   * @return true if IK solved, false otherwise.
   */
  bool computeIK6DOF(const Vector3d &tipPos, const Matrix3d &tipRot, const ArrayDofd &qIn,
                     std::list<ArrayDofd> &solution) const
  {
    solution.clear();
    std::list<double> distance;
    for (int i = 0; i < Configuration::Count; ++i)
    {
      ArrayDofd _solution;
      if (computeIK6DOF(tipPos, tipRot, Configuration(i), qIn, _solution))
      {
        double _distance = ((_solution - qIn) * jointWeight_).square().sum();

        std::list<ArrayDofd>::iterator sol_it = solution.begin();
        std::list<double>::iterator dis_it = distance.begin();
        for (; dis_it != distance.end() && _distance > *dis_it; dis_it++, sol_it++);

        solution.insert(sol_it, _solution);
        distance.insert(dis_it, _distance);
      }
    }

    return solution.size() > 0;
  }

  /**
   * @brief Compute nearest for initial joint state IK solution.
   * @param tipPos - end-effector postion
   * @param tipRot - end-effector orientation
   * @param qIn - initial joint state
   * @param solution - best solution
   * @return true if IK solved, false otherwise.
   */
  bool computeIK6DOF(const Vector3d &tipPos, const Matrix3d &tipRot, const ArrayDofd &qIn, /*out*/ ArrayDofd &qOut) const
  {
    bool ret = false;
    double minDist = std::numeric_limits<double>::max();

    ArrayDofd qOutI;
    int n = 0;
    for (int i = 0; i < Configuration::Count; ++i)
    {
      if (computeIK6DOF(tipPos, tipRot, Configuration(i), qIn, qOutI))
      {
        double distI = ((qOutI - qIn) * jointWeight_).square().sum();

        if(distI < minDist)
        {
          minDist = distI;
          qOut = qOutI;
          ret = true;
          n++;
        }
      }
    }

    return ret;
  }
private:

  /**
   * @brief Check joint limits and shift values for best matches with initial joint states.
   * @param qIn - initial joint state
   * @param qOut - solution
   */
  bool checkJointLimits(const ArrayDofd &qIn, ArrayDofd &qOut) const
  {
    static const double _2pi = atan(1) * 8.0;

    for(int i=0; i<DOF; ++i)
    {
      double delta = qIn.coeff(i) - qOut.coeff(i);

      qOut(i) += round(fabs(delta) / _2pi) * (delta < 0 ? -_2pi : _2pi);

      while(qOut(i) < lowerLimit_[i]) qOut(i) += _2pi;
      while(qOut(i) > upperLimit_[i]) qOut(i) -= _2pi;

      if(qOut(i) < lowerLimit_[i] || qOut(i) > upperLimit_[i])
        return false;
    }

    return true;
  } 
private:

  std::vector<std::string> jointNames_;
  ArrayDofd jointWeight_;
  ArrayDofd lowerLimit_;
  ArrayDofd upperLimit_;
  union {
    double length_[DOF];
    struct { double l0, l1, l2, l3, l4, l5; };
  };

  Matrix3d baseRot;
  Vector3d basePos;
  
  Matrix3d tau_6_T;
  Vector3d L_6_T_T;
    
};


} //namespace kuka_kinematics

#endif
