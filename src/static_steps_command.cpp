/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <pal_locomotion_msgs/ExecFootStepsAction.h>
#include <pal_talos_walk_impl/talos_walk_utils.h>
#include <actionlib/client/simple_action_client.h>
#include <pal_locomotion_msgs/PushVelocityCommandsSrv.h>
#include <geometry_msgs/Twist.h>
#include <pal_locomotion/velocity_command_que.h>
#include <pal_ros_utils/tf_utils.h>
#include <random>

typedef actionlib::SimpleActionClient<pal_locomotion_msgs::ExecFootStepsAction> WalkingClient;

// Function to generate random values between a specific interval
double generateRandomValue(double min, double max)
{
  std::uniform_real_distribution<double> unif(min, max);
  std::default_random_engine re(std::random_device{}());
  return unif(re);
}

// Generate a waypoint from the current position to the final position giving the swing
// leg height. This is exactly what is doing the interpolator inside if no other trajectory specified.
std::vector<pal_locomotion::CartesianTrajectoryPoint> createSwingTrajectoryLeg(
    const Eigen::Isometry3d &initial_foot_pose, const Eigen::Isometry3d &target_foot_pose,
    const ros::Duration &swing_leg_duration, double swing_leg_height)
{
  // Generates a point between the initial foot position and the target foot position
  Eigen::Vector3d midpoint =
      (initial_foot_pose.translation() + target_foot_pose.translation()) / 2.0;
  midpoint.z() = initial_foot_pose.translation().z() + swing_leg_height;

  // Generates a orientation between the initial foot orientation and the target foot
  // orientation
  Eigen::Quaterniond midrot = Eigen::Quaterniond(initial_foot_pose.rotation())
                                  .slerp(0.5, Eigen::Quaterniond(target_foot_pose.rotation()));

  // Creates a pose from the orientation + position
  Eigen::Isometry3d midpose = createMatrix(midrot, midpoint);

  // Push the generated pose at the middle of the swing leg duration
  std::vector<pal_locomotion::CartesianTrajectoryPoint> waypoints;

  // The points in the trajectory could be specified as pose, pose + vel, pose + vel + accel.
  // Right now the default interpolator only uses pose + vel to create a Cubic spline continous
  // in velocity.
  // If no velocity specified it calculates the velocity accoring to Catmul-Rom interpolation.
  pal_locomotion::CartesianTrajectoryPoint pt(midpose, ros::Duration(swing_leg_duration.toSec() / 2.0));
  waypoints.push_back(pt);

  // The final point is added in the StaticWalk action so it doesn't needs to be added.
  return waypoints;
}

// Generate a list of WalkingStepCommand giving velocities
std::vector<pal_locomotion::WalkingStepCommand> generateVelocityCommands(
    size_t n_steps, const ros::Duration &ds_duration, const ros::Duration &ss_duration)
{
  std::vector<pal_locomotion::WalkingStepCommand> commands;

  // Set the first and last step with double support an zero vel's
  pal_locomotion::WalkingStepCommand ds_step(Eigen::Vector3d::Zero(),
                                             Eigen::Vector3d::Zero(), ds_duration,
                                             ss_duration, +pal_locomotion::SupporType::DS);

  commands.push_back(ds_step);

  for (size_t i = 0; i < n_steps; i++)
  {
    if (i % 2 == 0)
    {
      // Generate a single support step giving linear velocities in x and y for the left
      // leg.
      // If 0 velocity specified in the y direction it uses the default foot separation
      // from the BipedParameters
      // In this example we set random vel in x from [0, 0.15] which means that the robot
      // will always move forward.
      pal_locomotion::WalkingStepCommand ss_step(
          Eigen::Vector3d(generateRandomValue(0.0, 0.15), generateRandomValue(0.0, 0.1), 0.0),
          Eigen::Vector3d::Zero(), ds_duration, ss_duration,
          +pal_locomotion::SupporType::SS, +pal_locomotion::Side::LEFT);

      commands.push_back(ss_step);
    }
    else
    {
      // Generate a single support step giving linear velocities in x and y for the right
      // leg.
      // In this case the velocity in the y direction is negative because we want to
      // separate the legs.
      pal_locomotion::WalkingStepCommand ss_step(
          Eigen::Vector3d(generateRandomValue(0.0, 0.1), generateRandomValue(-0.1, 0.0), 0.0),
          Eigen::Vector3d::Zero(), ds_duration, ss_duration,
          +pal_locomotion::SupporType::SS, +pal_locomotion::Side::RIGHT);

      commands.push_back(ss_step);
    }
  }

  // Push the last DS step
  commands.push_back(ds_step);

  return commands;
}

// Generate a list of WalkingStepCommand giving poses
std::vector<pal_locomotion::WalkingStepCommand> generatePoseCommands(
    size_t n_steps, const ros::Duration &ds_duration, const ros::Duration &ss_duration,
    double swing_leg_height)
{
  std::vector<pal_locomotion::WalkingStepCommand> commands;

  // Since the step poses are globally expressed respect odom we need to get the actual tf
  // of the robot.
  // In this example we will start with the left foot so we get the global right foot tf.
  Eigen::Isometry3d global_transform =
      pal::getTransform("odom", "right_sole_link", ros::Duration(3.0));

  // Set the first and last step with double support an zero vel's
  pal_locomotion::WalkingStepCommand ds_step(Eigen::Vector3d::Zero(),
                                             Eigen::Vector3d::Zero(), ds_duration,
                                             ss_duration, +pal_locomotion::SupporType::DS);

  commands.push_back(ds_step);

  for (size_t i = 0; i < n_steps; i++)
  {
    // In this example we generate local_transformations and multiply it by the previous
    // global transform (stance leg)
    // to get the desired transformation of the swing leg globally i.e respect odom
    pal_locomotion::Side side;
    std::vector<pal_locomotion::CartesianTrajectoryPoint> waypoints;
    if (i % 2 == 0)
    {
      // When the swing leg is the right leg we move 0.1 m in x and y direction and rotate
      // 0.2 rad/s respect the left foot
      global_transform = global_transform * createMatrix(Eigen::Vector3d(0, 0, 0.2),
                                                         Eigen::Vector3d(0.1, 0.2, 0));
      side = pal_locomotion::Side::LEFT;

      // In this case we don't specify any waypoint. Internally the static walk action
      // already generates a waypoint as in the createSwingTrajectoryLeg
    }
    else
    {
      // Get the initial swing leg pose
      Eigen::Isometry3d initial_swing_leg_pose =
          pal::getTransform("odom", "right_sole_link", ros::Duration(3.0));

      // When the swing leg is the left leg we move 0.15 m in x and y direction with zero
      // rotation.
      // Locally the y direction should be negative because we are going in opposite way
      // to the axis direction.
      global_transform = global_transform * createMatrix(Eigen::Vector3d(0, 0, 0),
                                                         Eigen::Vector3d(0.15, -0.2, 0));
      side = pal_locomotion::Side::RIGHT;

      // Generate a set of waypoints to define a trajectory of the swing leg.
      waypoints = createSwingTrajectoryLeg(initial_swing_leg_pose, global_transform,
                                           ss_duration, swing_leg_height);
    }

    // Create the footsteps and push it in the list.
    pal_locomotion::WalkingStepCommand ss_step(global_transform, ds_duration, ss_duration,
                                               side, waypoints);

    commands.push_back(ss_step);
  }
  // Push the last DS step
  commands.push_back(ds_step);

  return commands;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_steps_command");
  ros::NodeHandle nh;

  const size_t n_steps = 5;
  const ros::Duration ds_time = ros::Duration(2.0);
  const ros::Duration ss_time = ros::Duration(2.0);
  const double swing_leg_height = 0.08;

  ROS_INFO("Starting node");

  // In all the use cases when sending a desired footsep in tterms of velocity or pose,
  // this pose is clamped for safety to be inside a certain limits
  // Those limits are specified in the biped parameters as:
  // - step_saggital_bounds: [-0.1, 0.18] (Bound in the x axis)
  // - step_coronal_bounds: [-0.16, 0.45] (Bound in the y axis)
  // - step_rotational_bounds: [-0.1, 0.3] (Bound in the rotation axis)

  // 1. Send a list of footsteps using an action server.
  {
    // Create a walking simple action client
    WalkingClient walking_ac(nh, "/biped_walking_dcm_controller/footsteps_execution");

    ROS_INFO("Waiting for action client");
    bool ready = walking_ac.waitForServer(ros::Duration(10.0));

    if (!ready)
    {
      ROS_ERROR("Walking action is not ready");
      return (-1);
    }

    pal_locomotion_msgs::ExecFootStepsGoal new_goal;

    // The action expects a VelocityCommandMsg,
    // But for simplicity we usually create a vector of WalkingStepCommand and then we
    // convert it to VelocityCommandMsg
    std::vector<pal_locomotion::WalkingStepCommand> commands =
        generateVelocityCommands(n_steps, ds_time, ss_time);


    for (size_t i = 0; i < commands.size(); i++)
    {
      // Convert from WalkingStepCommand -> VelocityCommandMsg
      pal_locomotion_msgs::VelocityCommandMsg command_msg =
          pal_locomotion::convert(commands[i]);
      new_goal.commands.push_back(command_msg);
    }

    // Send the goal
    ROS_INFO("Sending goal");
    walking_ac.sendGoal(new_goal);

    // Wait for the action to be finished once all steps have been executed
    ROS_INFO("Waiting for result");
    bool finished = walking_ac.waitForResult(
        ros::Duration((n_steps + 2) * (ds_time.toSec() + ss_time.toSec()) * 2.5));

    if (!finished)
    {
      ROS_ERROR("Walking action dit not finish");
      return (-1);
    }
  }

  ROS_INFO("Action steps finished successfully");

  // 2. Send a list of footsteps using a service
  {
    // Create a service client
    ros::ServiceClient walking_srv =
        nh.serviceClient<pal_locomotion_msgs::PushVelocityCommandsSrv>(
            "/biped_walking_dcm_controller/push_velocity_commands");

    ROS_INFO("Waiting for service");
    bool ready = walking_srv.waitForExistence(ros::Duration(10.0));

    if (!ready)
    {
      ROS_ERROR("Walking service is not ready");
      return (-1);
    }

    // Service walking msg that expects (request) a vector of VelocityCommandMsg
    pal_locomotion_msgs::PushVelocityCommandsSrv srv_msg;

    // For simplicity we create a vector of WalkingStepCommand and then we convert it to
    // VelocityCommandMsg
    std::vector<pal_locomotion::WalkingStepCommand> commands =
        generatePoseCommands(n_steps, ds_time, ss_time, swing_leg_height);

    for (size_t i = 0; i < commands.size(); i++)
    {
      // Convert from WalkingStepCommand -> VelocityCommandMsg
      pal_locomotion_msgs::VelocityCommandMsg command_msg =
          pal_locomotion::convert(commands[i]);
      srv_msg.request.commands.push_back(command_msg);
    }

    // Call service
    ROS_INFO("Calling service");
    if (!walking_srv.call(srv_msg))
    {
      ROS_ERROR("Error calling walking service");
      return (-1);
    }
  }

  // Since it is a non-blocking service we sleep until the execution is finished.
  // The sleep time ideally should be (n_steps + 2) * (ds_time.toSec() + ss_time.toSec()).
  // But since internally it keeps going the foot down until a certain contact is
  // detected, and the COM is
  // close to a specific region the time is higher than the expected one.
  ros::Duration((n_steps + 2) * (ds_time.toSec() + ss_time.toSec())).sleep();

  ROS_INFO("Service steps finished successfully");

  // 3. Publish a certain velocity
  {
    // Create a publisher
    ros::Publisher walking_pub =
        nh.advertise<geometry_msgs::Twist>("/biped_walking_dcm_controller/cmd_vel", 1);

    // Fill a geometry_msgs::Twist msg with the desired velocity.
    // linear.x -> Moves the robot forward / backward
    // linear.y -> Moves the robot laterally. If angular velocity specified it has to be
    // zero.
    // linear.y -> Has to be zero
    // angular.x -> Has to be zero
    // angular.y -> Has to be zero
    // angular.z -> Rotational velocity

    // Internally the velocity is integrated with the swing time => 0.05m * 3m/s => 0.15
    // m/step
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.05;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = -0.1;

    ros::Time start_time = ros::Time(0);
    ros::Time time = start_time;
    ros::Duration dt = ros::Duration(0.1);

    // Publish a set of commands that are stored in the queue
    ROS_INFO("Publishing command in a topic");
    while ((time - start_time) < ros::Duration(15.0))
    {
      walking_pub.publish(vel_msg);
      ros::spinOnce();
      dt.sleep();
      time += dt;
    }
    // The robot will keep going until it has no more commands in the queue
  }

  ROS_INFO("Subscriber steps finished successfully");

  return (0);
}
