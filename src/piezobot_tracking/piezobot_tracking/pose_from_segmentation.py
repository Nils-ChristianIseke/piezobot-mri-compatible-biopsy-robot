from .analyze_segmentation import get_object_pose
import nibabel as nib
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import math
import numpy as np
from transforms3d.euler import euler2quat
import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class PoseFromSegmentation(Node):
    def __init__(self):
        super().__init__("InferenceNodennUnet")
        self.segmentation_result_subscriber = self.create_subscription(
            String,
            "/segmentation_result",
            self.segmentation_result_callback,
            10,
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.node = Node

    def segmentation_result_callback(self, msg):
        self.path_to_segmentation_result = msg.data
        nifti_image = nib.load(self.path_to_segmentation_result)
        pose = get_object_pose(nifti_image, {"needle_holder": 5})
        pose_mri = TransformStamped()
        pose_mri.header.frame_id = "mri_origin"
        pose_mri.header.stamp = self.get_clock().now().to_msg()
        pose_mri._child_frame_id = "needle_holder"
        pose_mri.transform.translation.x = pose["needle_holder"][0][0] / 1000
        pose_mri.transform.translation.y = pose["needle_holder"][0][1] / 1000
        pose_mri.transform.translation.z = pose["needle_holder"][0][2] / 1000
        orientation = euler2quat(
            pose["needle_holder"][1][0],
            pose["needle_holder"][1][1],
            pose["needle_holder"][1][2],
        )

        pose_mri.transform.rotation.x = orientation[0]
        pose_mri.transform.rotation.y = orientation[1]
        pose_mri.transform.rotation.z = orientation[2]
        pose_mri.transform.rotation.w = orientation[3]
        self.transform_broadcaster.sendTransform(pose_mri)
        self.get_logger().info(
            "Published pose of needle holder in MRI frame%s" % pose_mri
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseFromSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
