import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import torch
from nnunetv2.inference.predict_from_raw_data import nnUNetPredictor
from std_msgs.msg import String
import os


class PredictionnUnet(Node):
    path_to_prediction = ""
    path_to_images = ""
    os.environ[
        "nnUNet_results"
    ] = "/workspaces/piezobot_pc_workspace/src/piezobot_tracking/nnUNet_reults"

    def __init__(self):
        super().__init__("PredictionnnUnet")
        self.logger = self.get_logger()
        self.mri_image_subscriber = self.create_subscription(
            String, "/mri_image_path", self.mri_image_callback, 10
        )
        self.nnunet_prediction_publisher = self.create_publisher(
            String, "/segmentation_result", 10
        )
        self.predictor = nnUNetPredictor(
            tile_step_size=0.5,
            use_gaussian=True,
            use_mirroring=True,
            perform_everything_on_device=False,
            device=torch.device("cpu", 0),
            verbose=False,
            verbose_preprocessing=False,
            allow_tqdm=True,
        )
        self.predictor.initialize_from_trained_model_folder(
            "/workspaces/piezobot_pc_workspace/src/piezobot_tracking/nnUNet_reults/Dataset001_NeedlePhantomV1/nnUNetTrainer__nnUNetPlans__3d_fullres",
            use_folds=(0, 1, 2, 3, 4),
            checkpoint_name="checkpoint_final.pth",
        )

    def prediction(self):
        self.predictor.predict_from_files(
            self.path_to_images,
            self.path_to_prediction,
            save_probabilities=False,
            overwrite=True,
            num_processes_preprocessing=2,
            num_processes_segmentation_export=2,
            folder_with_segs_from_prev_stage=None,
            num_parts=1,
            part_id=0,
        )

    def mri_image_callback(self, path_to_images_msg):
        self.path_to_images = path_to_images_msg.data
        self.logger.info("received message" + self.path_to_images)
        self.path_to_prediction = os.path.join(
            "/workspaces/piezobot_pc_workspace/src/piezobot_tracking/nnunet_vs",
            "output",
        )
        self.prediction()
        path_to_prediction_msg = String()
        path_to_prediction_msg.data = os.path.joint(
            self.path_to_prediction, "mri.nii.gz"
        )
        self.nnunet_prediction_publisher.publish(path_to_prediction_msg)


def main(args=None):
    rclpy.init(args=args)

    node = PredictionnUnet()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# ros2 topic pub --once /mri_imgae_topic geometry_msgs/msg/Point "{x: 1.0, y: 2.0, z: 3.0}"
