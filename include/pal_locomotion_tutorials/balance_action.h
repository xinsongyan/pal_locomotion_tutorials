/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef BALANCING_ACTION_H
#define BALANCING_ACTION_H

#include <pal_locomotion/biped_controller.h>
#include <pal_locomotion/state_machine/walking_action_base.h>
// For realtime publisher and subscriber
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include<cmath>
//#include <icp_walking_sim/icp_walking.h>

namespace pal_locomotion
{
struct BalanceActionParameters : public ariles::ConfigurableBase
{
  BalanceActionParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    sin_freq_z_ = 1.0;
    sin_amp_z_ = 0.0;
    sin_freq_y_ = 1.0;
    sin_amp_y_ = 0.0;
    sin_freq_x_ = 1.0;
    sin_amp_x_ = 0.0;
      Kp_ = 1.0;
      Kd_ = 0.1;
  }

#define ARILES_SECTION_ID "BalanceActionParameters"
#define ARILES_CONSTRUCTOR BalanceActionParameters
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(sin_freq_x)                                                    \
  ARILES_ENTRY_(sin_freq_y)                                                     \
  ARILES_ENTRY_(sin_freq_z)                                                             \
  ARILES_ENTRY_(sin_amp_x)                                                             \
  ARILES_ENTRY_(sin_amp_y)                                                             \
  ARILES_ENTRY_(sin_amp_z)
#include ARILES_INITIALIZE

  double sin_freq_z_, sin_amp_z_;
  double sin_freq_y_, sin_amp_y_;
  double sin_freq_x_, sin_amp_x_;
  double Kp_, Kd_;  //com feedback gains
};

class BalanceAction : public WalkingActionBase
{
public:
  BalanceAction() : time_(0.0)
  {
  }

  BalanceAction(ros::NodeHandle &nh, BController *bController);
  virtual ~BalanceAction();

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time);

  /// This function is called every cycle of the loop, until the end of the Action()
  bool cycleHook(const ros::Time &time);

  /// Return "true" if the Action has to be stopped. The default implementation use time
  /// to stop the action;
  bool isOverHook(const ros::Time &time);

  /// when isOver()=true, this function is called and the action is removed from the
  /// queue.
  bool endHook(const ros::Time &time);

private:
  eVector2 computePcmp(double K, double omega, const eVector2 &icp_act,
                       const eVector2 &icp_des, const eVector2 &icp_des_vel);

  Eigen::Vector2d computeActICP(double omega, const Eigen::Vector2d &com,
                                const Eigen::Vector2d &com_d,
                                const Eigen::Vector2d &local_coord);

  ros::NodeHandle nh_;
  BController *bc_;
  eVector3 ini_target_;
  ddynamic_reconfigure::DDynamicReconfigurePtr ddr_;
  BalanceActionParameters params_;
  double time_;

  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> com_states_pub_;
  double n_com_states_;
  eVector3 current_com_pos_;
};
}

#endif  // BALANCING_ACTION_H
