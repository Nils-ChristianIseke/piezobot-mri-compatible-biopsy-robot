# piezobot_description
This ROS2 package contains all descriptions files of Piezobot, an MRI compatible robot actuated with piezomotors.
## Structure
### launch
- `view_piezobot.launch.py` starts rviz2 and displays the robot in it

### meshes

#### collision
Here you can store simplified versions of the meshes, which are used for collision detection (Currently empty, meshes from directory visual are used for collision checking).

#### visual
Contains the meshes of the robot. (base_link -> link6)

### rviz
`piezobot.rviz` is the config file of rviz2

### urdf
Contains the URDF of the robot. The basic URDF was created by exporting it with the Solidworks URDF Exporter. To get a more modular approach the exported URDF was manually split up to make it more modular

`common.xacro` Defines some standart objects that might be usefull
`piezobot.urdf.xacro` Main URDF that adds all URDF together

#### piezobot
`piezobot_macro.ros2_control.xacro`: Contains the ros2 controls interface available for the robot.
`piezobot_macro.xacro`: Defines the links and joints of the robot



 