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
            ) = self.extract_poses(msg.data, self.needle_holder_id, False)

            needle_position, needle_orientation = self.extract_poses(
                msg.data, self.needle_id
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
        self.get_logger().info(str(centroid))
        return centroid, orientation_vector

    def create_transform(self, position, orientation):
        pose = TransformStamped()
        pose.header.frame_id = "mri_origin"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.child_frame_id = "needle_holder_prediction"

        (
            pose.transform.translation.x,
            pose.transform.translation.y,
            pose.transform.translation.z,
        ) = (
            position / 1000
        )
        orientation_matrix = self.calculate_orientation_matrix(orientation)
        quat_orientation = mat2quat(orientation_matrix)

        pose.transform.rotation.x = quat_orientation[1]
        pose.transform.rotation.y = quat_orientation[2]
        pose.transform.rotation.z = quat_orientation[3]
        pose.transform.rotation.w = quat_orientation[0]

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
    main()


# import numpy as np
# import nibabel as nib
# from scipy.ndimage import label
# from skspatial.objects import Line, Points, Vector
# from nibabel.affines import apply_affine
# from transforms3d.quaternions import mat2quat

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import TransformStamped
# from tf2_ros.transform_broadcaster import TransformBroadcaster


# class PoseFromSegmentation(Node):
#     def __init__(self):
#         super().__init__("InferenceNodennUnet")
#         self.segmentation_result_subscriber = self.create_subscription(
#             String,
#             "/segmentation_result",
#             self.segmentation_result_callback,
#             10,
#         )
#         self.transform_broadcaster = TransformBroadcaster(self)
#         self.node = Node
#         self.needle_holder_id = 5
#         self.needle_id = 1
#         self.needle_holder_pose = {}
#         self.needle_pose = {}

#     def segmentation_result_callback(self, msg):
#         self.path_to_segmentation_result = msg.data
#         self.needle_holder_position = self.extract_poses(
#             msg.data, self.needle_holder_id
#         )[0]
#         self.needle_orienation = self.extract_poses(msg.data, self.needle_id, False)[1]
#         pose_needle_holder_mri = TransformStamped()
#         pose_needle_holder_mri.header.frame_id = "mri_origin"
#         pose_needle_holder_mri.header.stamp = self.get_clock().now().to_msg()
#         pose_needle_holder_mri._child_frame_id = "needle_holder_prediction"
#         pose_needle_holder_mri.transform.translation.x = (
#             self.needle_holder_position[0] / 1000
#         )
#         pose_needle_holder_mri.transform.translation.y = (
#             self.needle_holder_position[1] / 1000
#         )
#         pose_needle_holder_mri.transform.translation.z = (
#             self.needle_holder_position[2] / 1000
#         )
#         x_axis_needle = self.needle_orienation
#         anterior_axis_mri = np.array([0, 1, 0])
#         cosinus = np.dot(x_axis_needle, anterior_axis_mri)
#         if cosinus > 0:
#             x_axis_needle = -x_axis_needle

#         help_vector = deepcopy(x_axis_needle)
#         help_vector[0] += 1
#         help_vector = help_vector.unit()
#         if np.array_equal(help_vector, x_axis_needle):
#             help_vector[1] += 1
#             help_vector = help_vector.unit()
#             help_vector = Vector(help_vector)

#         y_axis = Vector(np.cross(help_vector, x_axis_needle)).unit()
#         z_axis = Vector(np.cross(x_axis_needle, y_axis)).unit()
#         orientation_matrix = np.column_stack((x_axis_needle, y_axis, z_axis))
#         orientation = mat2quat(orientation_matrix)
#         pose_needle_holder_mri.transform.rotation.x = orientation[1]
#         pose_needle_holder_mri.transform.rotation.y = orientation[2]
#         pose_needle_holder_mri.transform.rotation.z = orientation[3]
#         pose_needle_holder_mri.transform.rotation.w = orientation[0]
#         self.transform_broadcaster.sendTransform(pose_needle_holder_mri)
#         self.get_logger().info(
#             "Published pose of needle holder in MRI frame%s" % pose_needle_holder_mri
#         )

#     def extract_poses(self, path_to_image, label_id, needle_holder_complete=True):
#         nifti_image = nib.load(path_to_image)
#         if needle_holder_complete == True:
#             segmentation = self.merge_clusters(
#                 nifti_image, self.needle_id, self.needle_holder_id
#             )
#         else:
#             segmentation = nifti_image.get_fdata()

#         segmentation_voxel_coordinates = np.argwhere(segmentation == label_id)
#         if len(segmentation_voxel_coordinates) == 0:
#             error_message = "No segmentation found for label {}. Will not extract pose. \
#                 Check the segmentation_results of nnUnet".format(
#                 label_id
#             )
#             self.get_logger().info(error_message)
#             raise ValueError(error_message)
#         else:
#             segmentation_mri = apply_affine(
#                 nifti_image.affine, segmentation_voxel_coordinates
#             )
#             points_mri = Points(segmentation_mri)
#             orientation_vector_ras_plus = Line.best_fit(points_mri).vector
#             centroid_ras_plus = points_mri.centroid()
#         return [centroid_ras_plus, orientation_vector_ras_plus]

#     def merge_clusters(self, nifi, needle_id, needle_holder_id):
#         segmentaion = nifi.get_fdata()
#         segmentaion[segmentaion == needle_id] = needle_holder_id
#         segmentaion[segmentaion != needle_holder_id] = 0
#         labeled_array, num_features = label(segmentaion)
#         sizes = np.bincount(labeled_array.ravel())[1:]
#         largest_component = np.argmax(sizes) + 1
#         segmentaion[labeled_array != largest_component] = nifi.get_fdata()[
#             labeled_array != largest_component
#         ]
#         return segmentaion


# def main(args=None):
#     rclpy.init(args=args)
#     node = PoseFromSegmentation()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
