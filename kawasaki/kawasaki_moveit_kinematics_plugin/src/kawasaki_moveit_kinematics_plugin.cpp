/*
 * Copyright (c) 2013, Bauman Moscow State Technical University
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/*
 * Author: Sachin Chitta, David Lu!!, Ugo Cupcic, Igor Kalevatykh
 */

#include <kawasaki_moveit_kinematics_plugin/kawasaki_moveit_kinematics_plugin.h>


using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;


namespace kawasaki_kinematics
{

KawasakiKinematicsMoveitPlugin::KawasakiKinematicsMoveitPlugin() :
    active_(false), dimension_(6)
{
}

bool KawasakiKinematicsMoveitPlugin::isActive()
{
  if(active_)
    return true;
  return false;
}

bool KawasakiKinematicsMoveitPlugin::initialize(const std::string& robot_description,
                                                const std::string& group_name,
                                                const std::string& base_frame,
                                                const std::string& tip_frame,
                                                double search_discretization)
{
  ROS_INFO("KawasakiKinematicsMoveitPlugin initializing.");
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);


  // Get URDF XML
  std::string urdf_xml, full_urdf_xml;
  ros::NodeHandle node_handle;
  ros::NodeHandle private_handle("~"+group_name);
  ROS_INFO_STREAM("Private handle registered under " << private_handle.getNamespace());
  node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
  node_handle.searchParam(urdf_xml,full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");
  std::string result;
  if (!node_handle.getParam(full_urdf_xml, result)) {
    ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }
  // Load and Read Models
  if (!loadModel(result)) {
    ROS_FATAL("Could not load models!");
    return false;
  }

  urdf::Model robot_model;
  robot_model.initString(result);

  // Build Solvers
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  ik_solver_pos_.reset(new KawasakiIKSolver(robot_model, base_frame, tip_frame));
  active_ = true;
  return true;
}

bool KawasakiKinematicsMoveitPlugin::loadModel(const std::string &xml)
{
  urdf::Model robot_model;
  KDL::Tree tree;

  if (!robot_model.initString(xml)) {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree)) {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(base_frame_, tip_frame_, kdl_chain_)) {
    ROS_ERROR("Could not initialize chain object");
    return false;
  }
  if (!readJoints(robot_model)) {
    ROS_FATAL("Could not read information about the joints");
    return false;
  }
  return true;
}

bool KawasakiKinematicsMoveitPlugin::readJoints(urdf::Model &robot_model)
{
  const double BOUNDS_EPSILON = 1e-5;

  dimension_ = 0;
  // get joint maxs and mins
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_frame_);
  boost::shared_ptr<const urdf::Joint> joint;
  while (link && link->name != base_frame_) {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (!joint) {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
      ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
      dimension_++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  joint_min_.resize(dimension_);
  joint_max_.resize(dimension_);
  joint_names_.resize(dimension_);
  link = robot_model.getLink(tip_frame_);
  if(link)
    link_names_.push_back(tip_frame_);

  unsigned int i = 0;
  while (link && i < dimension_) {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
      ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

      float lower, upper;
      int hasLimits;
      if ( joint->type != urdf::Joint::CONTINUOUS ) {
        if(joint->safety) {
          lower = joint->safety->soft_lower_limit+BOUNDS_EPSILON; 
          upper = joint->safety->soft_upper_limit-BOUNDS_EPSILON;
        } else {
          lower = joint->limits->lower+BOUNDS_EPSILON;
          upper = joint->limits->upper-BOUNDS_EPSILON;
        }
        hasLimits = 1;
      } else {
        lower = -M_PI;
        upper = M_PI;
        hasLimits = 0;
      }
      int index = dimension_ - i - 1;

      joint_min_.data[index] = lower;
      joint_max_.data[index] = upper;
      joint_names_[index] = joint->name;
      i++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  return true;
}

int KawasakiKinematicsMoveitPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < joint_names_.size(); i++) {
    if (joint_names_[i] == name)
      return i;
  }
  return -1;
}

int KawasakiKinematicsMoveitPlugin::getKDLSegmentIndex(const std::string &name) const
{
  int i=0; 
  while (i < (int)kdl_chain_.getNrOfSegments()) {
    if (kdl_chain_.getSegment(i).getName() == name) {
      return i+1;
    }
    i++;
  }
  return -1;
}

bool KawasakiKinematicsMoveitPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                                   const std::vector<double> &ik_seed_state,
                                                   std::vector<double> &solution,
                                                   moveit_msgs::MoveItErrorCodes &error_code,
                                                   const kinematics::KinematicsQueryOptions &options) const
{
  if(!active_)
  {
    ROS_ERROR("KawasakiKinematicsMoveitPlugin: kinematics not active. getPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for (size_t i = 0; i < dimension_; i++)
    jnt_pos_in(i) = ik_seed_state[i];

  int ik_valid = ik_solver_pos_->CartToJnt(jnt_pos_in, pose_desired, jnt_pos_out);

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(unsigned int i=0; i < dimension_; i++)
      solution[i] = jnt_pos_out(i);

    error_code.val = error_code.SUCCESS;
    return true;
  }

  ROS_DEBUG("An IK solution could not be found");   
  error_code.val = error_code.NO_IK_SOLUTION;
  return false;
}

bool KawasakiKinematicsMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                      const std::vector<double> &ik_seed_state,
                                                      double timeout,
                                                      std::vector<double> &solution,
                                                      moveit_msgs::MoveItErrorCodes &error_code,
                                                      const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool KawasakiKinematicsMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                      const std::vector<double> &ik_seed_state,
                                                      double timeout,
                                                      const std::vector<double> &consistency_limits,
                                                      std::vector<double> &solution,
                                                      moveit_msgs::MoveItErrorCodes &error_code,
                                                      const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool KawasakiKinematicsMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                      const std::vector<double> &ik_seed_state,
                                                      double timeout,
                                                      std::vector<double> &solution,
                                                      const IKCallbackFn &solution_callback,
                                                      moveit_msgs::MoveItErrorCodes &error_code,
                                                      const kinematics::KinematicsQueryOptions &options) const
{
  if (!active_)
  {
    ROS_ERROR("KawasakiKinematicsMoveitPlugin: kinematics not active. searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  KDL::Frame pose_desired;
  //tf::PoseMsgToKDL(ik_pose, pose_desired);
  tf::poseMsgToKDL(ik_pose, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  std::vector<KDL::JntArray> jnt_pos_out_vec;
  jnt_pos_in.resize(dimension_);
  for (size_t i = 0; i < dimension_; i++)
    jnt_pos_in(i) = ik_seed_state[i];
  
  int ik_valid = ik_solver_pos_->CartToJnt(jnt_pos_in, pose_desired, jnt_pos_out_vec);

  if(ik_valid >= 0)
  {
    std::vector<double> solution_local(dimension_);

    for(size_t i=0; i<jnt_pos_out_vec.size(); ++i)
    {
      for(unsigned int j=0; j < dimension_; j++)
        solution_local[j] = jnt_pos_out_vec[i](j);

      solution_callback(ik_pose,solution_local,error_code);
      if(error_code.val == error_code.SUCCESS)
      {
        solution = solution_local;
        ROS_DEBUG_STREAM("Solved after " << i+1 << " iterations");
        return true;
      }      
    }
  }

  ROS_DEBUG("An IK that satisifes the constraints and is collision free could not be found");   
  error_code.val = error_code.NO_IK_SOLUTION;
  return false;
}

bool KawasakiKinematicsMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                      const std::vector<double> &ik_seed_state,
                                                      double timeout,
                                                      const std::vector<double> &consistency_limits,
                                                      std::vector<double> &solution,
                                                      const IKCallbackFn &solution_callback,
                                                      moveit_msgs::MoveItErrorCodes &error_code,
                                                      const kinematics::KinematicsQueryOptions &options) const
{
   return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code);
}

bool KawasakiKinematicsMoveitPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                                   const std::vector<double> &joint_angles,
                                                   std::vector<geometry_msgs::Pose> &poses) const
{
  if (!active_)
  {
    ROS_ERROR("KawasakiKinematicsMoveitPlugin: kinematics not active. getPositionFK");
    return false;
  }

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  jnt_pos_in.resize(dimension_);
  for (size_t i = 0; i < dimension_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  poses.resize(link_names.size());
  
  bool valid = true;
  for(unsigned int i=0; i < poses.size(); i++)
  {
    if(fk_solver_->JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >=0)
    {
      //tf::PoseKDLToMsg(p_out,poses[i]);
      tf::poseKDLToMsg(p_out, poses[i]);
    }
    else
    {
      ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

const std::vector<std::string>& KawasakiKinematicsMoveitPlugin::getJointNames() const
{
  if (!active_)
  {
    ROS_ERROR("KawasakiKinematicsMoveitPlugin: kinematics not active.");
  }

  return joint_names_;
}

const std::vector<std::string>& KawasakiKinematicsMoveitPlugin::getLinkNames() const
{
  if (!active_)
  {
    ROS_ERROR("KawasakiKinematicsMoveitPlugin: kinematics not active.");
  }

  return link_names_;
}

} // namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kawasaki_kinematics::KawasakiKinematicsMoveitPlugin, kinematics::KinematicsBase);
