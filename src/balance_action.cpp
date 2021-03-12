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
  ddr_->RegisterVariable(&params_.sin_amp_y_, "amplitude_y", 0.0, 0.085);
  ddr_->RegisterVariable(&params_.sin_freq_x_, "frequency_x", 0.0, 2.0);
  ddr_->RegisterVariable(&params_.sin_amp_x_, "amplitude_x", 0.0, 0.05);
  ddr_->RegisterVariable(&params_.Kp_x_, "Kp_x", 0.0, 30.0);
  ddr_->RegisterVariable(&params_.Kd_x_, "kd_x", 0.0, 10.0);
  ddr_->RegisterVariable(&params_.Kp_y_, "Kp_y", 0.0, 30.0);
  ddr_->RegisterVariable(&params_.Kd_y_, "kd_y", 0.0, 10.0);
  ddr_->RegisterVariable(&params_.Kp_z_, "Kp_z", 0.0, 30.0);
  ddr_->RegisterVariable(&params_.Kd_z_, "kd_z", 0.0, 10.0);
  ddr_->publishServicesTopics();

  // For realtime publisher
  n_com_states_ = 6;
  com_states_pub_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nh, "com_states", 1));

  desired_motions_buffer_.writeFromNonRT(default_motion_command_);
  sub_command_ = nh.subscribe<std_msgs::Float64MultiArray>("motionCommand", 1, &BalanceAction::commandCallback, this);

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

  std::vector<double>& desired_motionCommand = *desired_motions_buffer_.readFromRT();

  Eigen::Vector3d desired_pos;
  Eigen::Vector3d desired_vel;
  Eigen::Vector3d desired_acc;

  desired_pos.x() = params_.sin_amp_x_*sin(2*M_PI* time_*params_.sin_freq_x_);
  // desired_pos.y() = params_.sin_amp_y_*sin(2*M_PI* time_*params_.sin_freq_y_);
  desired_pos.y() = desired_motionCommand[2]*sin(2*M_PI* desired_motionCommand[3]);
  desired_pos.z() = params_.sin_amp_z_*sin(2*M_PI* time_*params_.sin_freq_z_) +  bc_->getParameters()->z_height_;

  desired_vel.x() = params_.sin_amp_x_*2*M_PI*params_.sin_freq_x_*cos(2*M_PI* time_*params_.sin_freq_x_);
  // desired_vel.y() = params_.sin_amp_y_*2*M_PI*params_.sin_freq_y_*cos(2*M_PI* time_*params_.sin_freq_y_);
  desired_vel.y() = desired_motionCommand[2]*2*M_PI*desired_motionCommand[3]*cos(2*M_PI* time_*desired_motionCommand[3]);
  desired_vel.z() = params_.sin_amp_z_*2*M_PI*params_.sin_freq_z_*cos(2*M_PI* time_*params_.sin_freq_z_);

  desired_acc.x() = -params_.sin_amp_x_*pow(2*M_PI*params_.sin_freq_x_, 2)*sin(2*M_PI* time_*params_.sin_freq_x_);
  // desired_acc.y() = -params_.sin_amp_y_*pow(2*M_PI*params_.sin_freq_y_, 2)*sin(2*M_PI* time_*params_.sin_freq_y_);
  desired_acc.y() = -desired_motionCommand[2]*pow(2*M_PI*desired_motionCommand[3], 2)*sin(2*M_PI* time_*desired_motionCommand[3]);
  desired_acc.z() = -params_.sin_amp_z_*pow(2*M_PI*params_.sin_freq_z_, 2)*sin(2*M_PI* time_*params_.sin_freq_z_);

  if (com_states_pub_ && com_states_pub_->trylock())
  {   
      current_com_pos_ = bc_->getActualCOMPosition();
      com_states_pub_->msg_.data.resize(n_com_states_);
      com_states_pub_->msg_.data[0] = current_com_pos_.x();
      com_states_pub_->msg_.data[1] = current_com_pos_.y();
      com_states_pub_->msg_.data[2] = current_com_pos_.z();
      com_states_pub_->msg_.data[3] = desired_pos.x();
      com_states_pub_->msg_.data[4] = desired_pos.y();
      com_states_pub_->msg_.data[5] = desired_pos.z();
      com_states_pub_->unlockAndPublish();
  }
  // Our actual COP and desired Pcmp are at the origin of the coordinate system as we just
  // wanted to balance the system mainting them at that point
  // Eigen::Vector2d act_COP = coordinate_system_2d.translation();
  // Eigen::Vector2d desiredPcmp = coordinate_system_2d.translation() + Eigen::Vector2d(dx, dy);
  // Eigen::Vector2d desiredPcmp_vel = Eigen::Vector2d::Zero();
  // desire_z = bc_->getParameters()->z_height_ + desire_z;

  // double omega = sqrt(bc_->getParameters()->gravity_ / z);
  Eigen::Vector2d actCOM = bc_->getActualCOMPosition2d();
  Eigen::Vector2d actCOMvel = bc_->getActualCOMVelocity2d();

  // Compute the ICP & Pcmp and next operations
  // Eigen::Vector2d actICP =
  //     this->computeActICP(omega, actCOM, actCOMvel, coordinate_system_2d.translation());
  // Eigen::Vector2d Pcmp = this->computePcmp(bc_->getParameters()->icp_gain_, omega, actICP, desiredPcmp, desiredPcmp_vel);

  // Here we do work in local cooridnates for accel and later converert it to global
  // for the computation for desired COM_vel and COM, we use actual ones as the for next control loop the desired is expected to be actual
  double dt = bc_->getControllerDt().toSec();
  current_com_pos_ = bc_->getActualCOMPosition();
  
  desired_acc.x() = desired_acc.x() + params_.Kp_x_* (desired_pos.x() - current_com_pos_.x()) + params_.Kd_x_*(desired_vel.x() - bc_->getActualCOMVelocity().x());
  desired_acc.y() = desired_acc.y() + params_.Kp_y_* (desired_pos.y() - current_com_pos_.y()) + params_.Kp_y_*(desired_vel.y() - bc_->getActualCOMVelocity().y());
  desired_acc.z() = desired_acc.z() + params_.Kp_z_* (desired_pos.z() - current_com_pos_.z()) + params_.Kp_z_*(desired_vel.z() - bc_->getActualCOMVelocity().z());

  // Eigen::Vector2d desiredCOM_acc = params_.Kp_*(desiredCOM - bc_->getActualCOMPosition()) + params_.Kd_*(desiredCOM_vel - bc_->getActualCOMVelocity()) + 
  //                                  omega * omega* ((actCOM - coordinate_system_2d.translation()) - (Pcmp-coordinate_system_2d.translation()));
  // Eigen::Vector2d desiredCOM_vel = actCOMvel + (desiredCOM_acc *dt);
  // Eigen::Vector2d desiredCOM = actCOM + (desiredCOM_vel * dt) + (0.5 * desiredCOM_acc * dt * dt);

  // Set the desired COM attributes to the biped controller
  bc_->setDesiredCOMPosition(eVector3(desired_pos.x(), desired_pos.y(), desired_pos.z())); // It's only for debugging
  bc_->setDesiredCOMVelocity(eVector3(desired_vel.x(), desired_vel.y(), desired_vel.z())); // It's only for debugging
  bc_->setDesiredCOMAcceleration(eVector3(desired_acc.x(), desired_acc.y(), desired_acc.z())); // This is for the control point

  // Set the ICP and COP attributes to the biped controller
  // All the below are only for debugging, but not used for the control point of view.
  // bc_->setDesiredICP(eVector3(desiredPcmp.x(), desiredPcmp.y(), 0.));
  // bc_->setDesiredCOPReference(eVector3(act_COP.x(), act_COP.y(), 0.));
  // bc_->setDesiredCOPComputed(eVector3(Pcmp.x(), Pcmp.y(), 0.0));
  // bc_->setActualICP(eVector3(actICP.x(), actICP.y(), 0.0));
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

void BalanceAction::commandCallback(const std_msgs::Float64MultiArrayConstPtr& msg){
  desired_motions_buffer_.writeFromNonRT(msg->data);
}