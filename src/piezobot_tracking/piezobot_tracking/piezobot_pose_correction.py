#!/usr/bin/env python3

import sys
import math

from geometry_msgs.msg import Pose

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
from tf2_ros.transform_broadcaster import TransformBroadcaster


class TfListener(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.link_6_frame = "link6"
        self.needle_holder = "needle_holder"
        self.base_link = "base_link"
        self.world = "world"

        self.get_logger().info(
            "Transforming from {} to {}".format(self.needle_holder, self.link_6_frame)
        )

        self.publisher = self.create_publisher(Pose, "corrected_pose", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # self.cmd_ = Twist()
        # self.publisher_ = self.create_publisher(
        #     Twist, "{}/cmd_vel".format(self.second_name_), 10
        # )
        self.timer = self.create_timer(0.33, self.timer_callback)  # 30 Hz = 0.333s
        self.tfb_ = TransformBroadcaster(self)

    def timer_callback(self):
        try:
            needle_holder_base_link = self._tf_buffer.lookup_transform(
                self.needle_holder, self.base_link, rclpy.time.Time()
            )
            #     self.tfb_.sendTransform(needle_holder_base_link)
            link_6_base_link = self._tf_buffer.lookup_transform(
                self.link_6_frame, self.world, rclpy.time.Time()
            )

            pos_needle_holder_x = needle_holder_base_link.transform.translation.x
            pos_needle_holder_y = needle_holder_base_link.transform.translation.y
            pos_needle_holder_z = needle_holder_base_link.transform.translation.z
            link_5_base_link_x = link_6_base_link.transform.translation.x
            link_5_base_link_y = link_6_base_link.transform.translation.y
            link_5_base_link_z = link_6_base_link.transform.translation.z

            err_x = pos_needle_holder_x - link_5_base_link_x
            err_y = pos_needle_holder_y - link_5_base_link_y
            err_z = pos_needle_holder_z - link_5_base_link_z

            corrected_pose_x = link_5_base_link_x + err_x
            corrected_pose_y = link_5_base_link_y + err_y
            corrected_pose_z = link_5_base_link_z + err_z
            corrected_pose = Pose()
            corrected_pose.position.x = link_6_base_link.transform.translation.y
            corrected_pose.position.y = link_6_base_link.transform.translation.x
            corrected_pose.position.z = link_6_base_link.transform.translation.z
            corrected_pose.orientation.x = link_6_base_link.transform.rotation.x
            corrected_pose.orientation.y = link_6_base_link.transform.rotation.y
            corrected_pose.orientation.z = link_6_base_link.transform.rotation.z
            corrected_pose.orientation.w = link_6_base_link.transform.rotation.w
            print(corrected_pose)
            print(link_6_base_link)
            self.publisher.publish(corrected_pose)

            #     tf_base_link_link_5 = self._tf_buffer.lookup_transform(
            #         self.target_frame, self.actual_frame, rclpy.time.Time()
            #     )
            #     self.cmd_.linear.x = math.sqrt(
            #         trans.transform.translation.x**2 + trans.transform.translation.y**2
            #     )
            #     self.cmd_.angular.z = 4 * math.atan2(
            #         trans.transform.translation.y, trans.transform.translation.x
            #     )
            #     self.publisher_.publish(self.cmd_)
        except LookupException as e:
            self.get_logger().error("failed to get transform {} \n".format(repr(e)))


def main(argv=None):
    rclpy.init(args=None)
    node = TfListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
