INPUT_FOLDER="/workspaces/piezobot_pc_workspace/src/piezobot_tracking/mri_images/Dataset220_NeedlePhantomV1"
#OUTPUT_FOLDER="/workspace/nnUNet_inference/Dataset001_NeedlePhantomV1/inference"
OUTPUT_FOLDER="/workspaces/piezobot_pc_workspace/src/piezobot_tracking/nnUnet_output"
#OUTPUT_FOLDER_PP="/workspace/nnUNet_inference/Dataset001_NeedlePhantomV1/postprocessed"
OUTPUT_FOLDER_PP="/workspace/mri_images/test_output_pp"
nnUNetv2_predict -d Dataset001_NeedlePhantomV1 -i $INPUT_FOLDER -o $OUTPUT_FOLDER -f  0 1 2 3 4 -tr nnUNetTrainer -c 3d_fullres -p nnUNetPlans

# nnUNetv2_apply_postprocessing -i $OUTPUT_FOLDER -o $OUTPUT_FOLDER_PP -pp_pkl_file /workspace/nnUNet_results/Dataset001_NeedlePhantomV1/nnUNetTrainer__nnUNetPlans__3d_fullres/crossval_results_folds_0_1_2_3_4/postprocessing.pkl -np 8 -plans_json /workspace/nnUNet_results/Dataset001_NeedlePhantomV1/nnUNetTrainer__nnUNetPlans__3d_fullres/crossval_results_folds_0_1_2_3_4/plans.json
