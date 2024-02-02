S

1. ros2 launch piezobot_bringup piezobot.launch.py 
2. ros2 launch piezobot_moveit_config move_group.launch.py
3. ros2 run piezobot_tracking get_ik_solution
4. ros2 run piezobot_tracking nnUnet 
5. ros2 run piezobot_tracking pose_from_segmentation
6. ros2 run piezobot_tracking calculate_correction_pose 






ros2 topic pub --once /piezobot/pose_correction geometry_msgs/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, pose: {position: {x: 0.3, y: 0.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.8509035, w: 0.525322}}}"



To send commands to the forwar_position controller: 
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.
- 0.
- 0.
- 0.
- 0
- 0"

FOR Real hardware:

ros2 launch piezobot_bringup piezobot.launch.py use_mock_hardware:=false

sudo chmod 666 /dev/ttyUSB0



pip list | grep matplotlib and apt list --installed | grep matplotlib
<!-- apt remove python3-matplotlib -->
pip uninstall matplotlib