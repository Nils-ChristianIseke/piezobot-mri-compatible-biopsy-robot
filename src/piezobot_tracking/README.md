# piezobot_tracking
This ros2 package takes care of making the predection on the mri data. Postprocessing of the predection, and broadcasting the predicted pose of the needleholder to the tf-tree of the robot.

## nnUnet_input
In this directory the mri image aquired should be saved. It's name should be input.nii.gz. Only one, the most recent image, should be stored here.

## nnUnet_output
In this directory the output of nnUnets prediction is stored. It is called output.nii.gz

## nnUnet_results
In this directory the trained model of nnUnet is stored, which is used to predict the segmentations.

## piezobot_tracking

- `piezobot_pose_correction.py`: Calculates the pose_error between the target pose of the robot and the from the segmentation of the predicted_pose (from which we assume to be the actual pose of the robot) calculated pose. The difference is added to the target pose, to get a new target pose, which minimizes the pose_error.