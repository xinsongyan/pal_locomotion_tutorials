/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <pal_locomotion/static_step_executor.h>
#include <math_utils/math_utils.h>
#include <pal_ros_utils/tf_utils.h>
#include <pal_utils/popl.hpp>

using namespace popl;
using namespace pal_locomotion;

BipedStepList createStepList(const eMatrixHom& global_transform, int steps_num,
                             double step_size, double foot_separation,
                             double switch_time = 0.3, double stance_time = 0.9)
{
  ros::Duration switch_duration(switch_time);
  ros::Duration stance_duration(stance_time);

  BipedStepList sl;

  pal_locomotion::Side side;
  for (int i = 1; i < steps_num; ++i)
  {
    double dist_y = foot_separation;
    if (i % 2 == 0)
    {
      side = Side::RIGHT;
      dist_y *= -1;
    }
    else
    {
      side = Side::LEFT;
    }
    eMatrixHom step_pose =
        createMatrix(eVector3(0., 0., 0.), eVector3(step_size * i, dist_y, 0.0));
    Step step(switch_duration, stance_duration, global_transform * step_pose, side);
    ROS_ERROR_STREAM((global_transform * step_pose).translation().transpose());
    sl.push_back(step);
  }
  side = (side._value == Side::RIGHT) ? Side::LEFT : Side::RIGHT;
  foot_separation = (side._value == Side::RIGHT) ? (foot_separation * -1.0) : foot_separation;
  eMatrixHom last_step_pose = createMatrix(
      eVector3(0., 0., 0.), eVector3(step_size * (steps_num - 1), foot_separation, 0.0));
  Step last_step(switch_duration, stance_duration, global_transform * last_step_pose, side);
  sl.push_back(last_step);

  return sl;
}

int main(int argc, char** argv)
{
  double ds_duration;
  double swing_duration;
  double x_step_dist;
  int step_num;
  double foot_separation;

  Switch help_option("h", "help", "produce help message");

  Value<double> ds_duration_option("d", "ds_duration", "ds duration option", 0., &ds_duration);
  Value<double> swing_duration_option("u", "swing_time", "swing leg time option", 0.,
                                      &swing_duration);
  Value<int> step_num_option("s", "step_num", "Number of steps option", 5, &step_num);
  Value<double> x_step_dist_option("x", "x_dist", "Distance in the x direction", 0.15,
                                   &x_step_dist);
  Value<double> foot_separation_option("y", "foot_separation", "Foot separation", 0.1,
                                       &foot_separation);

  OptionParser op("Allowed options");
  op.add(help_option);
  op.add(ds_duration_option);
  op.add(swing_duration_option);
  op.add(step_num_option);
  op.add(x_step_dist_option);
  op.add(foot_separation_option);

  op.parse(argc, argv);

  // print auto-generated help message
  if (help_option.isSet() || !ds_duration_option.isSet() ||
      !swing_duration_option.isSet() || !x_step_dist_option.isSet())
  {
    std::cout << op << "\n";
    return 0;
  }

  if ((foot_separation < 0.1) || (foot_separation > 0.25))
  {
    ROS_ERROR_STREAM("The foot separation provided is dangerous. It should be between [0.1, 0.25]");
  }
  if (fabs(x_step_dist) > 0.3)
  {
    ROS_ERROR_STREAM("The x distance provided is higher than 0.3");
  }

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  eMatrixHom global_transform = pal::getTransform("odom", "base_footprint", ros::Duration(3.0));

  BipedStepList step_list =
      createStepList(global_transform, step_num, x_step_dist, foot_separation);

  StaticStepExecutor executor(nh, ros::Duration(ds_duration),
                              ros::Duration(swing_duration), step_list);
  ROS_INFO_STREAM("Will execute motion for " << step_list.size() << " static steps!");
  executor.execute();
}
