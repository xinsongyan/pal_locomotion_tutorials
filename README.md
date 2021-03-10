# PAL Locomotion Tutorials

This package is intended to serve as a tutorial with worked examples and utilities for a better understanding of the DCM controller implemented at PAL Robotics.

This package includes:
* The [balance_action](./src/balance_action.cpp): an implementation of an action of the DCM controller to control the Center of Mass (CoM) of the robot making it describe a sinusoidal wave.
* The [static_steps_command](./src/static_steps_command.cpp): is a description of the different interfaces available to send footsteps to the robot when performing quasi-static walking.
* The [static_steps_execution_node](./src/static_steps_execution_node.cpp): that creates a list of footsteps to command the robot to move on a straight line.

## Balance action

The balance action is a specific action from the state machine inside the DCM controller. The [talos_pal_locomotion](https://gitlab/control/talos_pal_locomotion) package contains a brief explanation of the state machine and the available actions inside the DCM controller.

The balance action shows the capabilities of the robot to maintain balance while moving its CoM along each coordinate axis. The implementation is based on the paper [*Bipedal walking control based on Capture Point dynamics*][bipedal_walking_control_link], by Englsberger, Ott, Roa, Albu-Schäfer and Hirzinger. Within the balance action node, the CoM of the robot is commanded to move on a sinusoidal trajectory of a user-provided frequency and amplitude. The robot maintains balance while performing the motion.

### Code of Balance Action

* The [balance_action.cpp](./src/balance_action.cpp) file is an implementation of the action by controlling the dynamics of the [paper cited above][bipedal_walking_control_link] in a plugin, that uses the information from the Biped controller.
* The [pal_locomotion_tutorials_plugins.cpp](./src/pal_locomotion_tutorials_plugins.cpp) file uses the plugin library to export the class from the package.
* The [balance_action_node.cpp](./src/balance_action_node.cpp) file is an implementation of a node that pushes the created plugin into the `biped_walking_dcm_controller` and starts the action.

#### Execution of Balance Action

* In simulation
    1. To launch the action, first launch the simulation:
        ```
        roslaunch talos_gazebo talos_gazebo.launch
        ```
    2. then launch the DCM controller to start the biped controller:
        ```
        roslaunch talos_pal_locomotion talos_dcm_walking_controller.launch estimator:=kinematic_estimator_params
        ```
* In the real robot

    To launch the demo on the **real robot**, 

    1. Deploy this package on the robot, and then reboot the robot
    2. Launch the default controllers on the robot, once the robot is in walking pose, place the robot on ground
        ```
        roslaunch talos_controller_configuration default_controllers.launch
        ```
    3. Kill the default controllers
    4. Launch the DCM Controller on the robot (refer to the handbook for the command)

Now, To launch the balance action node, run the following command:
```
rosrun pal_locomotion_tutorials balance_action_node
```

<span style="color:red">**Caution: When running in the robot, increase the limits in small fractions to avoid breaking the robot (or) to avoid completely destabilize the robot** </span>

Open `rqt_reconfigure` and then navigate to the following option `biped_walking_dcm_controller`->`balance control_params` as shown in the figure:

![Balance Control Parameters](./images/balance_action_rqt_reconf.png#center)

With the parameters in the `rqt_reconfigure`, set the desired frequency and amplitude of the desired sinusoidal motion in the desired direction. The below picture shows a glimpse of robot in the simulation performing the motion, while maintaining the balance. 

![TALOS Squat](./images/balance_action_talos_squat.png#center)

## Execution of Static Steps

The following nodes are an example of how to send footsteps to the robot by using an action, a service, a publisher or a library.

- [static_steps_command](./src/static_steps_command.cpp)
- [static_steps_execution_node](./src/static_steps_execution_node.cpp)

The first one uses the three ROS interfaces to send a list o footsteps (i.e action, service, publisher). In that example the footsteps are defined either as velocity commands or as a poses in the world. It also contains an example of how to define trajectories in Cartesian space for the swing leg.

![TALOS Static Steps Execution](./images/static_steps_execution_talos.png#center)

The second one uses the `StaticStepExecutor` library from pal_locomotion. The main difference with respect to the first node is that the previous one uses the `static_walk` action from [pal_locomotion_actions](https://gitlab/control/pal_locomotion_actions), while this node uses the `stand_one_leg_action, stand_down_leg_action` actions. In these actions there is no tolerance in the CoM considered to go to the next state, it is done by time.

**Parameters:**
- The parameter `d` (or) `ds_duration` is the transition duration of each commanded step.
- The parameter `u` (or) `swing_duration` is the swing duration of each commanded step.
- The parameter `s` (or) `step_num` is the number of steps to be executed.
- The parameter `x` (or) `x_dist` is footstep distance in the x direction.
- The parameter `y` (or) `foot_separation` is footstep distance in the y direction.

For example, to perform 5 steps (with the suggested parameters by the static_steps_execution_node):

* In simulation
    1. To launch the action, first launch the simulation:
        ```
        roslaunch talos_gazebo talos_gazebo.launch
        ```
    2. then launch the DCM controller to start the biped controller:
        ```
        roslaunch talos_pal_locomotion talos_dcm_walking_controller.launch estimator:=kinematic_estimator_params
        ```
    3. and run the static_steps_execution_node
        ```
        rosrun pal_locomotion_tutorials static_steps_execution_node -d 2.0 -u 2.0 -s 5 -x 0.15 -y 0.1
        ```

* In the robot
    1. Follow the procedure expained above to execute the balance action, and run
        ```
        rosrun pal_locomotion_tutorials static_steps_execution_node -d 2.0 -u 2.0 -s 5 -x 0.15 -y 0.1
        ```

[bipedal_walking_control_link]: https://ieeexplore.ieee.org/document/6094435
