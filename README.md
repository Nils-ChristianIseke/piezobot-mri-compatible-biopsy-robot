# MRI Based Visual Servoing 


## Get Started
Following Prerequisites must be met:
1. Docker Installation
2. VS Code Installation

To get started clone this repo: 
1. Clone this repository: git clone git@github.com:Nils-ChristianIseke/piezobot.git
2. Open VS Code, navigate to the cloned repository and open the container.
3. Install all necessary Dependencys with the task install dependencys
4. Build the project by 



sudo apt-get install libwiringpi-dev


TODO: Hardware Interface fertig implementieren. Schauen warum der PORT nicht geöffnet ist!


ros2 launch piezobot_bringup piezobot.launch.py use_mock_hardware:=false
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [-0.03, -0.03,0.02,0.02]}"



ros2 launch piezobot_bringup piezobot.launch.py robot_controller:=joint_trajectory_controller
ros2 launch piezobot_bringup test_joint_trajectory_controller.launch.py


ros2 launch piezobot_bringup piezobot.launch.py robot_controller:=joint_trajectory_controller use_mock_hardware:=false
ros2 launch piezobot_bringup test_joint_trajectory_controller.launch.py


ros2 launch piezobot_bringup piezobot.launch.py robot_controller:=forward_position_controller use_mock_hardware:=false

ros2 launch piezobot_bringup test_forward_position_controller.launch.py


sudo socat -d -d pty,raw,echo=0 "PTY,link=/dev/ttyUSB0
"


ros2 launch piezobot_moveit_config servo.launch.py



# For testing without real
ros2 launch piezobot_bringup piezobot.launch.py use_mock_hardware:=true
ros2 launch piezobot_moveit_config move_group.launch.py




# Need to specify controller, otherwise controller manager is not starting.
ros2 launch piezobot_bringup piezobot.launch.py robot_controller:=forward_position_controller use_mock_hardware:=false

ros2 launch piezobot_moveit_config move_group.launch.py

ros2 run intro_target target_fine


ros2 topic pub /introct/navigation/point_entry  geometry_msgs/msg/Point "{x: 0, y: 0, z: 0}"
 ros2 topic pub /introct/navigation/point_target  geometry_msgs/msg/Point "{x: 0, y: 0, z: 0}"



 intro_taget error [ERROR] [1688395368.220173820] [move_group_interface]: Pose for end-effector 'link6' not known.

 move_group-1] [INFO] [1688397749.493455736] [moveit_ros.current_state_monitor]: Didn't received robot state (joint angles) with recent timestamp within 1.000000 seconds.


 ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [-0.1, 0,0,0,0,0]}"# piezobot
