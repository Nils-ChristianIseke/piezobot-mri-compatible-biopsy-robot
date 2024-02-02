#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import TransformStamped
from .quaternion_calculations import quaternion_from_euler, quaternion_multiply
from transforms3d.quaternions import  quat2mat
import math
import numpy as np


class PoseCorrection(Node):
    def __init__(self):
        super().__init__("PoseCorrection")
        self.planned_pose_name = "link6"
        self.attained_pose_name = "needle_holder_prediction"
        self.base_link_name = "base_link"
        self.get_logger().info(
            "Transforming from {} to {}".format(
                self.attained_pose_name, self.planned_pose_name
            )
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.timer = self.create_timer(4, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.corrected_pose = PoseStamped()
        self.attained_pose_previous = TransformStamped()
        self.publisher = self.create_publisher(
            PoseStamped, "piezobot/pose_correction", 10
        )
        self.correction_quaterion = quaternion_from_euler(0, 0, 0)
        self.corrected_quaterion = quaternion_from_euler(0, 0, 0)
        self.error_x = 0
        self.error_y = 0
        self.error_z = 0
        self.publisher_result = self.create_publisher(String, "results", 10)

    def timer_callback(self):
        try:
            self.planned_pose = self._tf_buffer.lookup_transform(
                self.base_link_name, self.planned_pose_name, rclpy.time.Time()
            )

            self.attained_pose = self._tf_buffer.lookup_transform(
                self.base_link_name, self.attained_pose_name, rclpy.time.Time()
            )

        except LookupException as e:
            self.get_logger().error("failed to get transform {} \n".format(repr(e)))
            return

            # if (
            #     self.attained_pose_previous.transform.rotation
            #     != self.attained_pose.transform.rotation
            #     or self.attained_pose_previous.transform.rotation
            #     != self.attained_pose.transform.rotation
            # ):
        self.attained_pose_previous = self.attained_pose

        self.attained_pose_previous.transform.translation = (
            self.attained_pose.transform.translation
        )
        self.error_x = (
            self.planned_pose.transform.translation.x
            - self.attained_pose.transform.translation.x
        )
        self.corrected_pose.pose.position.x = (
            self.planned_pose.transform.translation.x + self.error_x
        )
        self.error_y = (
            self.planned_pose.transform.translation.y
            - self.attained_pose.transform.translation.y
        )
        self.corrected_pose.pose.position.y = (
            self.planned_pose.transform.translation.y + self.error_y
        )
        self.error_z = (
            self.planned_pose.transform.translation.z
            - self.attained_pose.transform.translation.z
        )
        self.corrected_pose.pose.position.z = (
            self.planned_pose.transform.translation.z + self.error_y
        )
        attained_quaternion_inverse = quaternion_from_euler(0, 0, 0)
        attained_quaternion_inverse[0] = -self.attained_pose.transform.rotation.w
        attained_quaternion_inverse[1] = self.attained_pose.transform.rotation.x
        attained_quaternion_inverse[2] = self.attained_pose.transform.rotation.y
        attained_quaternion_inverse[3] = self.attained_pose.transform.rotation.z

        planned_quaternion = quaternion_from_euler(0, 0, 0)
        planned_quaternion[0] = self.planned_pose.transform.rotation.w
        planned_quaternion[1] = self.planned_pose.transform.rotation.x
        planned_quaternion[2] = self.planned_pose.transform.rotation.y
        planned_quaternion[3] = self.planned_pose.transform.rotation.z

        self.correction_quaterion = quaternion_multiply(
            planned_quaternion, attained_quaternion_inverse
        )
        self.get_logger().info(
            "planned correction_quaterion: {}".format(self.correction_quaterion)
        )
        self.corrected_quaterion = quaternion_multiply(
            self.correction_quaterion, planned_quaternion
        )

        self.corrected_pose.header.frame_id = "base_link"
        self.corrected_pose.header.stamp = self.get_clock().now().to_msg()
        self.corrected_pose.pose.orientation.x = self.corrected_quaterion[1]
        self.corrected_pose.pose.orientation.y = self.corrected_quaterion[2]
        self.corrected_pose.pose.orientation.z = self.corrected_quaterion[3]
        self.corrected_pose.pose.orientation.w = self.corrected_quaterion[0]
        self.publisher.publish(self.corrected_pose)
        self.get_logger().info("corrected pose: {}".format(self.corrected_pose))
        self.publish_results()

    def publish_results(self):
        data_message = (
            "Attained_Position: {:.3f}, {:.3f}, {:.3f}, ".format(
                self.planned_pose.transform.translation.x,
                self.planned_pose.transform.translation.y,
                self.planned_pose.transform.translation.z,
            )
            + "Planned_Position: {:.3f}, {:.3f}, {:.3f}, ".format(
                self.attained_pose.transform.translation.x,
                self.attained_pose.transform.translation.y,
                self.attained_pose.transform.translation.z,
            )
            + "Total position Error in m: {:.3f}, ".format(
                math.sqrt(
                    math.pow(self.error_x, 2)
                    + math.pow(self.error_y, 2)
                    + math.pow(self.error_z, 2)
                )
            )
            + "Position Error in m per axis: {:.3f}, {:.3f}, {:.3f}, ".format(
                self.error_x, self.error_y, self.error_z
            )
            + "Attained_Orientation: {:.3f}, {:.3f}, {:.3f}, {:.3f}, ".format(
                self.planned_pose.transform.rotation.w,
                self.attained_pose.transform.rotation.x,
                self.attained_pose.transform.rotation.y,
                self.attained_pose.transform.rotation.z,
            )
            + "Planned_Orientation: {:.3f}, {:.3f}, {:.3f}, {:.3f}, ".format(
                self.attained_pose.transform.rotation.w,
                self.attained_pose.transform.rotation.x,
                self.attained_pose.transform.rotation.y,
                self.attained_pose.transform.rotation.z,
            )
            + "Orientation Error (rad): {:.3f}, ".format(
                self.calculate_deviation(self.attained_pose, self.planned_pose)
            )
            + "Corrected position: {:.3f}, {:.3f}, {:.3f}, ".format(
                self.corrected_pose.pose.position.x,
                self.corrected_pose.pose.position.y,
                self.corrected_pose.pose.position.z,
            )
            + "Corrected orientation: {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(
                self.corrected_pose.pose.orientation.w,
                self.corrected_pose.pose.orientation.x,
                self.corrected_pose.pose.orientation.y,
                self.corrected_pose.pose.orientation.z,
            )
        )

        self.publisher_result.publish(String(data=data_message))

    def calculate_deviation(self, transform1, transform2):
        rotation_matrix1 = quat2mat(
            [
                transform1.transform.rotation.x,
                transform1.transform.rotation.y,
                transform1.transform.rotation.z,
                transform1.transform.rotation.w,
            ]
        )
        rotation_matrix2 = quat2mat(
            [
                transform2.transform.rotation.x,
                transform2.transform.rotation.y,
                transform2.transform.rotation.z,
                transform2.transform.rotation.w,
            ]
        )

        x_axis1 = rotation_matrix1[:, 0]
        x_axis2 = rotation_matrix2[:, 0]

        angle_deviation = np.arccos(
            np.dot(x_axis1, x_axis2)
            / (np.linalg.norm(x_axis1) * np.linalg.norm(x_axis2))
        )
        return angle_deviation


def main(argv=None):
    rclpy.init(args=None)
    node = PoseCorrection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
