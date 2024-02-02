# Piezobot_bringup

This ros2 package contains configs and launch files to start different scenarios. It is based on the work


## launch
This directory contains the launch files of this package. By default they are assuming that now real hardware is connected.

- `piezobot.launch.py`
   -  Displays robot in rviz2 and send commands to the ros2 controllers
-  `piezobot_bringup piezobot_sim_gazebo_classic.launch.py`
   - Starts a Gazebo classic simulation: needed if you want to simulate stuff, using the old version of gazebo

-  `piezobot_bringup piezobot_sim_gazebo.launch.py`
   - Starts a Gazebo modern simulation: needed if you want to simulate stuff, using the new version of gazebo

- `piezobot_bringup test_forward_position_controller.launch.py`
  - Displays robot in rviz2, loads a forward_postion_controller and publishes data to the forward_position_controller 

- `piezobot_bringup test_joint_trajectory_controller.launch.py`
  - Displays robot in rviz2, loads a joint_trajectory_controller and publishes data to the joint_trajectory_controller 

## config
piezobot_controllers.yaml: Defines the available controllers, which are loaded by the controller manager
including the joints and state / commands interfaces

test_global_publishers.yaml:

## ros2_control
If you want to learn more about ros2_control and get a better understanding, follow its [documentatoin](https://control.ros.org/master/index.html).