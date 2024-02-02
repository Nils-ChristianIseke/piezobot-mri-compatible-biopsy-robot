# piezobot_moveit_config
This ROS2 package is in charge of launching moveit. It was created using the moveit_setup_assistant with the piezobot_description package.

## config

The config directory contains different config files, most important are:
- `initial_positions.yaml`: Defines the initial positions of the robot.
- `joint_limits.yaml`: Defines the joint limits of the robot
- `kinematics.yaml`: Defines which kinematic solver should be used for the robot
- `moveit_controllers.yaml`: Defines the moveit controllers
- `piezobot.srdf`: Semantic robot description, defines groups of joint for planning purposes, group states (e.g. home position), and self collisions

## launch

The launch directory contains different launch files, most important are:
- `move_group.launch.py` launches the move_group, enabling to use the power of moveit, calculating motion plans etc.
