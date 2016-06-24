/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012-2014, Igor Kalevatykh, Bauman Moscow State Technical University
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

#include <inttypes.h>
#include <stdio.h>
#include <time.h>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kawasaki_kinematics/kawasaki_ik.h>
using namespace kawasaki_kinematics;

class KawasakiIkTest : public testing::Test
{
public:
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  boost::shared_ptr<kawasaki_kinematics::KawasakiIK> ik_solver_;

  virtual void SetUp()
  {
    std::string root_frame = "base_link";
    std::string tip_frame = "link_6";
    boost::filesystem::path package_path(ros::package::getPath("kawasaki_description"));
    std::string xml_file = (package_path / "urdf" / "fs03n" / "fs03n.urdf").string();

    urdf::Model model;
    KDL::Chain chain;
    KDL::Tree tree;

    if(model.initFile(xml_file))
    {
      ik_solver_.reset(new kawasaki_kinematics::KawasakiIK(model, root_frame, tip_frame));
    }

    if(kdl_parser::treeFromFile(xml_file, tree))
    {
      if (tree.getChain(root_frame, tip_frame, chain))
      {
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain));
      }
    }
  }

  virtual void TearDown()
  {
  }
};

TEST_F(KawasakiIkTest, Initializing)
{
  ASSERT_TRUE(ik_solver_);
  ASSERT_TRUE(fk_solver_);
}

TEST_F(KawasakiIkTest, InverseKinematics)
{
  std::srand(std::time(0));

  for(int i=0; i<1000; ++i)
  {
    Eigen::VectorXd randVec(6);
    randVec << rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX;
    Eigen::VectorXd low = Eigen::VectorXd::Map(ik_solver_->lowerLimit().data(), KawasakiIK::DOF);
    Eigen::VectorXd up = Eigen::VectorXd::Map(ik_solver_->upperLimit().data(), KawasakiIK::DOF);
    Eigen::VectorXd q = low + (up - low).cwiseProduct(randVec);

    KDL::Frame frame;
    KDL::JntArray q_kdl(6); q_kdl.data = q;
    fk_solver_->JntToCart(q_kdl, frame);
    Eigen::Vector3d pos = Eigen::Vector3d::Map(&frame.p.data[0]);
    Eigen::Matrix3d rot = Eigen::Matrix3d::Map(&frame.M.data[0]);

    KawasakiIK::ArrayDofd q_in = KawasakiIK::ArrayDofd::Map(q.data());
    KawasakiIK::ArrayDofd q_out;

    Configuration cfg_in = ik_solver_->getConfig(q_in);

    EXPECT_TRUE(ik_solver_->computeIK6DOF(pos, rot, cfg_in, q_in, q_out))
        << "Tip: " << pos << std::endl << rot;

    EXPECT_LT((q_in - q_out).array().abs().maxCoeff(), 1e-8)
       << "In: " << q_in.transpose() << std::endl
       << "Out: " << q_out.transpose();

    EXPECT_TRUE(ik_solver_->computeIK6DOF(pos, rot, q_in, q_out))
	<< "Tip: " << pos << std::endl << rot;

    EXPECT_LT((q_in - q_out).array().abs().maxCoeff(), 1e-8)
       << "In: " << q_in.transpose() << std::endl
       << "Out: " << q_out.transpose();

    EXPECT_EQ(ik_solver_->getConfig(q_in), ik_solver_->getConfig(q_out))
       << "In: " << ik_solver_->getConfig(q_in).toString() << std::endl
       << "Out: " << ik_solver_->getConfig(q_out).toString();
  }  
}

TEST_F(KawasakiIkTest, InverseKinematicsStability)
{
  Eigen::VectorXd randVec(6);
  randVec << rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX;
  Eigen::VectorXd low = Eigen::VectorXd::Map(ik_solver_->lowerLimit().data(), KawasakiIK::DOF);
  Eigen::VectorXd up = Eigen::VectorXd::Map(ik_solver_->upperLimit().data(), KawasakiIK::DOF);
  Eigen::VectorXd q = low + (up - low).cwiseProduct(randVec);

  KDL::Frame frame;
  KDL::JntArray q_kdl(KawasakiIK::DOF); q_kdl.data = q;
  fk_solver_->JntToCart(q_kdl, frame);
  Eigen::Vector3d pos = Eigen::Vector3d::Map(&frame.p.data[0]);
  Eigen::Matrix3d rot = Eigen::Matrix3d::Map(&frame.M.data[0]);

  KawasakiIK::ArrayDofd q_in = KawasakiIK::ArrayDofd::Map(q.data());
  KawasakiIK::ArrayDofd q_out;

  for(int i=0; i<1000; ++i)
  {
    fk_solver_->JntToCart(q_kdl, frame);
    Eigen::Vector3d pos = Eigen::Vector3d::Map(&frame.p.data[0]);
    Eigen::Matrix3d rot = Eigen::Matrix3d::Map(&frame.M.data[0]);

    EXPECT_TRUE(ik_solver_->computeIK6DOF(pos, rot, q_in, q_out))
        << "Counter: " << i;

    q_kdl.data = Eigen::VectorXd::Map(q_out.data(), KawasakiIK::DOF);
  }

  EXPECT_LT((q_in - q_out).array().abs().maxCoeff(), 1e-8)
     << "In: " << q_in.transpose() << std::endl
     << "Out: " << q_out.transpose();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

