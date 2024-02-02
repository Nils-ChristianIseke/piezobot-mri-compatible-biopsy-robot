# piezobot_servo
This ros2 package is enabling the use of moveit_servo for the piezobot. With this it is possible, to send cartesian commands (e.g.: velocities) to the robot. Thus it is enabling visual servoing for the piezobot. To learn more about moveit_servo, click [here](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html#).

## launch
- `piezobot_servo.launch.py` launches the servo_node

## config
- `piezobot_simulated_config.yaml` Configuration file for the servo node. Define things like planning group, joint_limit_margis etc.

## src
- `servo_keyboard_input.cpp`Enables you to send cartesian commands to the piezobot via your keyboard.

