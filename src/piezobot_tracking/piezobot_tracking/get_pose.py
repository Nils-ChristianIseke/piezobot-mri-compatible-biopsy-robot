import numpy as np
import nibabel as nib
from scipy.ndimage import label
from skspatial.objects import Line, Points, Vector
from nibabel.affines import apply_affine
from transforms3d.quaternions import mat2quat

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster
import argparse


class PoseFromSegmentation(Node):
    def __init__(self):
        super().__init__("InferenceNodeUnet")
        self.segmentation_result_subscriber = self.create_subscription(
            String, "/segmentation_result", self.segmentation_result_callback, 10
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.needle_holder_id = 5
        self.needle_id = 1

    def segmentation_result_callback(self, msg):
        try:
            (
                needle_holder_position,
                needle_holder_orientation,
                a,
                b,
            ) = self.extract_poses(msg.data, self.needle_holder_id, True)

            self.get_logger().info(
                str(needle_holder_position, a, b, needle_holder_orientation)
            )

            needle_position, needle_orientation = self.extract_poses(
                msg.data, self.needle_holder_id
            )
            needle_holder_pose = self.create_transform(
                needle_holder_position, needle_orientation
            )
            self.transform_broadcaster.sendTransform(needle_holder_pose)
            self.get_logger().info(
                f"Published pose of needle holder in MRI frame: {needle_holder_pose}"
            )
        except ValueError as e:
            self.get_logger().info(str(e))

    def extract_poses(self, path_to_image, label_id, merge_clusters=False):
        nifti_image = nib.load(path_to_image)
        if merge_clusters:
            segmentation = self.merge_clusters(nifti_image)
        else:
            segmentation = nifti_image.get_fdata()

        segmentation_voxel_coordinates = np.argwhere(segmentation == label_id)
        if len(segmentation_voxel_coordinates) == 0:
            raise ValueError(
                f"No segmentation found for label {label_id}. Check the segmentation_results of nnUnet"
            )

        segmentation_mri = apply_affine(
            nifti_image.affine, segmentation_voxel_coordinates
        )
        points_mri = Points(segmentation_mri)
        orientation_vector = Line.best_fit(points_mri).vector
        centroid = points_mri.centroid()
        self.get_logger().info(str(orientation_vector))
        return (centroid, orientation_vector)

    def create_transform(self, position, orientation):
        pose = TransformStamped()
        pose.header.frame_id = "mri_origin"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.child_frame_id = "needle_holder_prediction"
        meter_per_mm = 0.001  # As ros uses meter as unit, we need to convert to m

        pose.transform.translation.x = position[0] * meter_per_mm
        pose.transform.translation.y = position[1] * meter_per_mm
        pose.transform.translation.z = position[2] * meter_per_mm

        orientation_matrix = self.calculate_orientation_matrix(orientation)
        quat_orientation = mat2quat(orientation_matrix)

        (
            pose.transform.rotation.x,
            pose.transform.rotation.y,
            pose.transform.rotation.z,
            pose.transform.rotation.w,
        ) = (quat_orientation[1:], quat_orientation[0])
        return pose

    def calculate_orientation_matrix(self, x_axis_needle):
        anterior_axis_mri = np.array([0, 1, 0])
        x_axis_needle = (
            -x_axis_needle
            if np.dot(x_axis_needle, anterior_axis_mri) > 0
            else x_axis_needle
        )

        help_vector = Vector(x_axis_needle + [1, 0, 0]).unit()
        if np.array_equal(help_vector, x_axis_needle):
            help_vector = Vector(
                [x_axis_needle[0], x_axis_needle[1] + 1, x_axis_needle[2]]
            ).unit()

        y_axis = Vector(np.cross(help_vector, x_axis_needle)).unit()
        z_axis = Vector(np.cross(x_axis_needle, y_axis)).unit()

        return np.column_stack((x_axis_needle, y_axis, z_axis))

    def merge_clusters(self, nifti_image):
        segmentation = nifti_image.get_fdata()
        segmentation[segmentation == self.needle_id] = self.needle_holder_id
        segmentation[segmentation != self.needle_holder_id] = 0
        labeled_array, num_features = label(segmentation)
        sizes = np.bincount(labeled_array.ravel())[1:]
        largest_component = np.argmax(sizes) + 1
        segmentation[labeled_array != largest_component] = nifti_image.get_fdata()[
            labeled_array != largest_component
        ]
        return segmentation


def main(args=None):
    rclpy.init(args=args)
    node = PoseFromSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script with flags")
    parser.add_argument(
        "--path_to_image",
        type=str,
        help="Path to directory containing images to analyze",
    )

    args = parser.parse_args()
    path_to_image_dir = args.path_to_image
