import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

class GetIKSolution(Node):
    def __init__(self):
        callback_client = ReentrantCallbackGroup()
        subscriber_cb_group = ReentrantCallbackGroup()
        super().__init__("get_ik_solution")
        self.client = self.create_client(
            GetPositionIK, "compute_ik", callback_group=callback_client
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.position_ik_request = GetPositionIK.Request()
        self.current_states = JointState()
        self.subscription = self.create_subscription(
            PoseStamped,
            "piezobot/pose_correction",
            self.calculate_ik_callback,
            10,
            callback_group=subscriber_cb_group,
        )
        self.subscription_jointStates = self.create_subscription(
            JointState, "joint_states", self.read_joint_states, 10
        )

        self.forward_position_commander = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )

        self.subscription

    def read_joint_states(self, msg):
        self.current_states = msg

    def calculate_ik_callback(self, msg):
        self.get_logger().info("Starting IK calculations")
        service_request = PositionIKRequest()
        service_request.group_name = "arm"
        service_request.ik_link_name = "link6"
        service_request.robot_state.joint_state.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        service_request.robot_state.joint_state.position = [0] * 6
        service_request.pose_stamped.pose.position = msg.pose.position
        service_request.pose_stamped.pose.orientation = msg.pose.orientation
        self.position_ik_request.ik_request = service_request
        response = self.client.call(self.position_ik_request)

        if response.solution.joint_state.position.tolist() == []:
            self.get_logger().info("No Solution Found")

        else:
            self.get_logger().info("Solution Found")
            joints_ik = response.solution.joint_state
            command_position_forward_commander = Float64MultiArray()
            command_position_forward_commander.data = joints_ik.position
            self.forward_position_commander.publish(command_position_forward_commander)


def main(args=None):
    rclpy.init(args=args)
    node = GetIKSolution()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("Beginning client, shut down with CTRL-C")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
