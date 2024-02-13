
# Structure of the Repository
This repo contains all necessary ros2 packages in the src directory.
Furthermore it contains, the mri images int the directory mri_images.
And the setup used to train nnUnet on the HPC in the directory high_performance_cluster


<p align="center">
<img src="https://github.com/Nils-ChristianIseke/piezobot-mri-compatible-biopsy-robot/assets/48475933/b23f74b0-789e-43f5-a3e9-70337cada2ed" width="500">
</p>
  Piezobot - MRI compatible biopsy robot, based on piezoelectric actuators and ceramic bearings.

<p align="center">
<img src="https://github.com/Nils-ChristianIseke/piezobot-mri-compatible-biopsy-robot/assets/48475933/8ee418ea-ec4c-461f-a7a7-38c800fa6aad" width="500">
</p>


3D Segmentation of an MRI from a needle phantom. The pose extraced from this segmentation is used to do look-and-move Visual Servoing of the needle. Needleholder (blue), Needle (red), Tumor (yellow), Tissue (green).


## MRI Based Visual Servoing 
This ros2 workspace enabled look-and-move visual servoing for the piezobot, an mri compatible biopsy
robot.

### Get Started
Following Prerequisites must be met:
1. Docker Installation
2. VS Code installation (recommended not obligatory)
3. nnU-netv2 installation

To get started clone this repo: 
1. Clone this repository: `git clone git@github.com:Nils-ChristianIseke/piezobot.git`
2. Open VS Code, navigate to the cloned repository and open the container moveit2_rolling.
3. Install all necessary dependencies with the vs code task 'install dependencies'.
4. Build the project with the task 'build'.


### Connecting the Robot to Your PC

- **Connect the robot to your PC** using the USB RS interface.

### Identifying the Correct Port

- Open a terminal on your PC and enter the following command to list devices connected via USB:

sudo dmesg | grep tty

- Look through the output for entries related to newly connected devices to **find the correct port
  name**.
- Allow access to the port via ```sudo chmod 666 serial_port_name```

### Updating the Robot Configuration

- Navigate to the robot's URDF file located at: ros2 package
src/piezobot_description/urdf/piezobot/piezobot_macro.ros2_control.xacro
- In this file, locate the parameter named `serial_port_name` and **replace it with the current port name**
 (the one you identified from the `dmesg` output).

### Manual Calibration Precaution

- **Please note**, the robot does not support automatic calibration to prevent potential damage to
  its cables or structure from exceeding the physical limits of its joints. Therefore before starting the Visual Servoing system, you neet to:

### Move the Robot to the Home Position

- Ensure the robot is in its home position before starting. You can verify the home position using the rviz
visualization tool by running: ros2 launch piezobot_bringup piezobot.launch.py
use_mock_hardware:=true.
The pose in which the robot is displayed in the beginning is the starting pose you need to achieve
on the physical robot.

- Use the arduino serial monitor to manually move the robot joints. For joints 1 and 2, use the jogging command
  `X(axis_number)J,(+-)1000,0,1000` to move them to their physical limits. To stop the motors, use the
  command `X(axis_number)S`.
- For joints 3 and 4, jog them to the zero position, indicated by a triangle pointing to a line for
  joint 3, and a line for joint 4.

### Setting the Encoders to Zero

- **After moving the joints to their respective positions, set the encoders for the joints to
  zero (X0~E0)**.

By following these steps, you'll have successfully connected your robot to your PC and manually
calibrated it for use. This process ensures the robot operates within safe limits and is ready for
further tasks.

Now you can start the Visual Servoing by:

1. `ros2 launch piezobot_bringup piezobot.launch.py use_mock_hardware:=false`
2. `ros2 launch piezobot_moveit_config move_group.launch.py`
3. `ros2 run piezobot_tracking get_ik_solution`
4. `ros2 run piezobot_tracking nnUnet` 
5. `ros2 run piezobot_tracking pose_from_segmentation`
6. `ros2 run piezobot_tracking calculate_correction_pose` 


Here the commands do the following:
1. Starting Robot Visualization and Ros2_control: Launch the robot visualization  in RViz along with ros2_control. 
    Begin the tf_publisher to broadcast the position of the needle holder relative to
    the pose of link_4. To adjust the published pose, modify the corresponding launch file as
    needed.

2. Activating Move Group for Piezobot:

    Start the move_group for the Piezobot to enable MoveIt capabilities. If a different inverse
    kinematics (IK) solver is preferred, update the kinematics.yaml file found in the config
    directory of the piezobot_moveit_config.

3. Utilizing MoveIt Service for Joint Values:

    Launch a client for the MoveIt service GetPositionIK to obtain joint values for the newly
    calculated (corrected) pose.

4. Running nnU-Net Prediction on MRI Images:

    Execute the nnU-Net prediction process on the captured MRI images to obtain segmentation
    results.

5. Extracting Pose from Segmentation:

    Derive the pose information from the segmentation output provided by the nnU-Net prediction.

6. Calculating Correction Pose:

    Compute the correction pose based on the extracted pose and any additional parameters or
    conditions required for precise positioning.


Now you can use the forward_position_controller to move to a starting position.

To send commands to the forward_position controller: 

```
ros2 topic pub --once
/forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.03
- 0.03
- 0.
- 0.
- 0
- 0.017"
```

As soon as the robot is at the new target position, start the biopsy, then make an MRI Scan of the scene,
remove the mri needle from the phantom, so that the robot can freely move,
crop the ROI of the mri image using MITK Workbench and send the path of the cropped scene
to the topic mri_image_path:
'ros2 topic pub  --once std_msgs/msg/String "{data: path_to_mri_image}"
This starts the Visual Servoing process, and which end the pose correction of the robot is done.

## The following parts can be found in this [repo](https://github.com/Nils-ChristianIseke/piezobot_mri)

## High Performance Cluster
The trained model can be found in high_performance_cluster/hd_oy280-nnUNetv2/nnUNet_results/Dataset001_NeedlePhantomV1/nnUNetTrainer__nnUNetPlans__3d_fullres/fold_0/checkpoint_final.pth,
for fold 0 and for the other folds in the according directories.
To use all folds to make a prediction, follow the information in inferece_instructions.txt
Please refer to the official documentation of nnUnetv2 for more information regarding, training new 
Datasets.
job.sh is a job that starts the training inside an enroot container. You need to adjust the referenced 
scripts to train on a new dataset.
Start training  with: sbatch --partition=dev_gpu_4\a100 job.sh}, label=lst:cluster_job

## MRI Images
### Evaluations
Contains the signal measurements for marker and phantom materials. If needed,
scan parameters can be read out from the according scans in the MITK files.
### MRI Measuerments 
Contains the acquired mri images.
Most important are: [TrainingData scans](mri_images/MRI_Measuerments/MRT_AREA_16_10/MRI_16_10/TEST_ROBOTER_23_10_05-14_29_28-DST-1_3_12_2_1107_5_2_18_141978/SEQUENCE_REGION_SIEMENS_SEQUENCES_20231016_132459_652000/WithNeedle), the scans on which the training
data was created, and [evaluationScans](mri_images/MRI_Measuerments/MRT_AREA_Evaluation_31_10) the scans on which
the orientation accuracy of the segmentation results was evaluated.
 If needed,
scan parameters can be read out from the metadata of the scans.
### Tumor Liver
Contains the annotated mri data that was used to create the tumor moldings.
