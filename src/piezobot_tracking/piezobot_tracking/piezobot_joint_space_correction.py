#!/usr/bin/env python3

import sys
import math

from geometry_msgs.msg import PoseStamped
from transforms3d.euler import quat2euler
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import TransformStamped
from std_msgs.msg import Float64MultiArray


class JointSpaceCorrection(Node):
    def __init__(self):
        super().__init__("PoseCorrection")
        self.planned_pose = "needle_holder"
        self.attained_pose = "needle_holder_prediction"
        self.link_1 = "link1"
        self.link_2 = "link2"
        self.link_3 = "link3"
        self.link_4 = "link4"
        self.world = "world"
        self.displacement_publisher = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.timer = self.create_timer(0.33, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.logger = self.get_logger()
        self.displacement_previous = ""

    def timer_callback(self):
        try:
            
            displacement = self._tf_buffer.lookup_transform(
                self.world, self.attained_pose, rclpy.time.Time()
            )
            if self.displacement_previous != displacement:
                self.displacement_previous = displacement
                discplacement_world = self._tf_buffer.lookup_transform(self.displacement)
                displacement.transform
                correction_x = -displacement.transform.translation.x
                correction_y = -displacement.transform.translation.y
                correction_z = -displacement.transform.translation.z
                self.logger.info(
                    "The following corrections will be send to the displacement_contorller "
                    + str(correction_x)
                    + " "
                    + str(correction_y)
                    + " "
                    + str(correction_z)
                )

                joint1_command = -correction_z
                joint2_command = -correction_x
                rotation = displacement.transform.rotation
                rotation.w = -rotation.w
                roll, pitch, yaw = quat2euler(
                    (rotation.w, rotation.x, rotation.y, rotation.z), axes="sxyz"
                )

                joint3_command = roll
                joint4_command = -yaw

                displacement_message = Float64MultiArray()
                displacement_message.data = [
                    joint1_command,
                    joint2_command,
                    joint3_command,
                    joint4_command,
                    0,
                    0,
                ]
                self.logger.info("Sending posecorrection to commander")
                self.displacement_publisher.publish(displacement_message)

        except LookupException as e:
            self.get_logger().error("failed to get transform {} \n".format(repr(e)))


def main(argv=None):
    rclpy.init(args=None)
    node = JointSpaceCorrection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
