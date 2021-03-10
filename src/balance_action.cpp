/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <pal_locomotion_tutorials/balance_action.h>
#include <math_utils/geometry_tools.h>
#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

using namespace pal_locomotion;
using namespace math_utils;

BalanceAction::BalanceAction(ros::NodeHandle &nh, BController *bController)
  :nh_(nh), time_(0.0)
{
  if (!configure(nh, bController, property_bag::PropertyBag()))
  {
    PAL_THROW_DEFAULT("problem configuring actoin");
  }
}

BalanceAction::~BalanceAction()
{
}

bool BalanceAction::configure(ros::NodeHandle &nh, BController *bController,
                              const property_bag::PropertyBag &parameters)
{
  bc_ = bController;
  ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(
      ros::NodeHandle(nh, "balance_control_params")));
  ddr_->RegisterVariable(&params_.sin_freq_z_, "frequency_z", 0.0, 2.0);
  ddr_->RegisterVariable(&params_.sin_amp_z_, "amplitude_z", 0.0, 0.08);
  ddr_->RegisterVariable(&params_.sin_freq_y_, "frequency_y", 0.0, 2.0);
  ddr_->RegisterVariable(&params_.sin_amp_y_, "amplitude_y", 0.0, 0.07);
  ddr_->RegisterVariable(&params_.sin_freq_x_, "frequency_x", 0.0, 2.0);
  ddr_->RegisterVariable(&params_.sin_amp_x_, "amplitude_x", 0.0, 0.05);
  ddr_->publishServicesTopics();
  return true;
}

bool BalanceAction::enterHook(const ros::Time &time)
{
  std::vector<Side> stance_id;
  stance_id.push_back(+Side::LEFT);
  stance_id.push_back(+Side::RIGHT);
  std::vector<Side> swingLegIds;
  bc_->setStanceLegIDs(stance_id);
  bc_->setSwingLegIDs(swingLegIds);

  bc_->setWeightDistribution(0.5);
  bc_->setActualSupportType(+SupporType::DS);

  eMatrixHom actualLeftFootPose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actualRightFootPose = bc_->getActualFootPose(+Side::RIGHT);

  ini_target_ = (actualLeftFootPose.translation() + actualRightFootPose.translation()) / 2.;

  return true;
}

bool BalanceAction::cycleHook(const ros::Time &time)
{
  // Get the actual pose of the foot, so that we can find the center point of the foot
  eMatrixHom act_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom act_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  // Create the local coordinate frame of reference
  eMatrixHom local_coordinate_frame =
      interpolateBetweenTransforms(act_left_foot_pose, act_right_foot_pose);

  eMatrixHom2d coordinate_system_2d;
  pal::convert(local_coordinate_frame, coordinate_system_2d);

  double dz = params_.sin_amp_z_*sin(2*M_PI* time_*params_.sin_freq_z_);
  double dy = params_.sin_amp_y_*sin(2*M_PI* time_*params_.sin_freq_y_);
  double dx = params_.sin_amp_x_*sin(2*M_PI* time_*params_.sin_freq_x_);

  // Our actual COP and desired Pcmp are at the origin of the coordinate system as we just
  // wanted to balance the system mainting them at that point
  Eigen::Vector2d act_COP = coordinate_system_2d.translation();
  Eigen::Vector2d desiredPcmp = coordinate_system_2d.translation() + Eigen::Vector2d(dx, dy);
  Eigen::Vector2d desiredPcmp_vel = Eigen::Vector2d::Zero();
  double z = local_coordinate_frame.translation().z() + bc_->getParameters()->z_height_ + dz;

  double omega = sqrt(bc_->getParameters()->gravity_ / z);
  Eigen::Vector2d actCOM = bc_->getActualCOMPosition2d();
  Eigen::Vector2d actCOMvel = bc_->getActualCOMVelocity2d();

  // Compute the ICP & Pcmp and next operations
  Eigen::Vector2d actICP =
      this->computeActICP(omega, actCOM, actCOMvel, coordinate_system_2d.translation());
  Eigen::Vector2d Pcmp = this->computePcmp(bc_->getParameters()->icp_gain_, omega, actICP, desiredPcmp, desiredPcmp_vel);

  // Here we do work in local cooridnates for accel and later converert it to global
  // for the computation for desired COM_vel and COM, we use actual ones as the for next control loop the desired is expected to be actual
  double dt = bc_->getControllerDt().toSec();
  Eigen::Vector2d desiredCOM_acc = omega * omega* ((actCOM - coordinate_system_2d.translation()) - (Pcmp-coordinate_system_2d.translation()));
  Eigen::Vector2d desiredCOM_vel = actCOMvel + (desiredCOM_acc *dt);
  Eigen::Vector2d desiredCOM = actCOM + (desiredCOM_vel * dt) + (0.5 * desiredCOM_acc * dt * dt);

  // Set the desired COM attributes to the biped controller
  bc_->setDesiredCOMPosition(eVector3(desiredCOM.x(), desiredCOM.y(), z)); // It's only for debugging
  bc_->setDesiredCOMVelocity(eVector3(desiredCOM_vel.x(), desiredCOM_vel.x(), 0.0)); // It's only for debugging
  bc_->setDesiredCOMAcceleration(eVector3(desiredCOM_acc.x(), desiredCOM_acc.y(), 0.0)); // This is for the control point

  // Set the ICP and COP attributes to the biped controller
  // All the below are only for debugging, but not used for the control point of view.
  bc_->setDesiredICP(eVector3(desiredPcmp.x(), desiredPcmp.y(), 0.));
  bc_->setDesiredCOPReference(eVector3(act_COP.x(), act_COP.y(), 0.));
  bc_->setDesiredCOPComputed(eVector3(Pcmp.x(), Pcmp.y(), 0.0));
  bc_->setActualICP(eVector3(actICP.x(), actICP.y(), 0.0));
  time_ += dt;
  return true;
}

bool BalanceAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

bool BalanceAction::endHook(const ros::Time &time)
{
  ROS_INFO_STREAM("Balance ds end hook, time: " << time.toSec());
  return true;
}


eVector2 BalanceAction::computePcmp(double K, double omega, const eVector2 &icp_act,
                                    const eVector2 &icp_des, const eVector2 &icp_des_vel)
{
  eVector2 Pcmp = icp_act - ((1 / omega) * icp_des_vel) + (K * (icp_act - icp_des));
  return Pcmp;
}

Eigen::Vector2d BalanceAction::computeActICP(double omega, const Eigen::Vector2d &com,
                                             const Eigen::Vector2d &com_d,
                                             const Eigen::Vector2d &local_coord)
{
  Eigen::Vector2d icp = (com - local_coord) + (com_d / omega);
  return (icp + local_coord);
}
