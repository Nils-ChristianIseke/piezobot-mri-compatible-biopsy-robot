/*
 Copyright 2023 Google LLC

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/robot_state.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/geometry_msgs/msg/point.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/geometry_msgs/msg/pose_stamped.hpp>


using moveit::planning_interface::MoveGroupInterface;

class PoseCorrection : public MoveGroupInterface
{
public:
  explicit PoseCorrection(rclcpp::Node::SharedPtr node)
  : MoveGroupInterface(node, "arm")
  {
    publisher_joint_goal_ = node->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "piezobot/moveit/joint_goal_fine", 10);

    subscriber_pose_goal_ = node->create_subscription<geometry_msgs::msg::Pose>(
      "piezobot/pose_goal", 10, [this](geometry_msgs::msg::Pose msg_ft)
      {callback_pose_goal(msg_ft);});

    pose_from_segmentation = node->create_subscription<geometry_msgs::msg::Pose>(
      "/pose_from_segmentation", 10, [this](geometry_msgs::msg::Pose msg_ft)
      {callback_pose_from_segmentation(msg_ft);});

    publisher_point_entry_rviz_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "rviz/corrected_goal", 10);

    publisher_point_target_rviz_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "rviz/needle_position", 10);
  }

private:
  geometry_msgs::msg::PoseStamped joint_group_positions = this->getCurrentPose();
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_joint_goal_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_pose_goal_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_from_segmentation;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_point_entry_rviz_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_point_target_rviz_;

  double millimeter_to_meter_ = 1000;

  void callback_pose_goal(geometry_msgs::msg::Pose msg_pose)
  {
    this->setPoseTarget(msg_pose);
  }

  void callback_pose_from_segmentation(geometry_msgs::msg::Pose msg_pt)
  {
    geometry_msgs::msg::Pose target_pose = this->getPoseTarget().pose;
    // NOch nicht sicher mit Vorzeichen und multiplikativer Inverse
    target_pose.position.x += (target_pose.position.x - msg_pt.position.x);
    target_pose.position.y += (target_pose.position.y - msg_pt.position.y);
    target_pose.position.z += (target_pose.position.z - msg_pt.position.z);
    Eigen::Quaterniond target_pose_orientaion_quaternion;
    target_pose_orientaion_quaternion.x() = target_pose.orientation.x;
    target_pose_orientaion_quaternion.y() = target_pose.orientation.y;
    target_pose_orientaion_quaternion.z() = target_pose.orientation.z;
    target_pose_orientaion_quaternion.w() = target_pose.orientation.w;
    target_pose_orientaion_quaternion.normalize();
    target_pose_orientaion_quaternion.inverse();

    Eigen::Quaterniond aruco_pose_orientaion_quaternion;
    aruco_pose_orientaion_quaternion.x() = msg_pt.orientation.x;
    aruco_pose_orientaion_quaternion.y() = msg_pt.orientation.y;
    aruco_pose_orientaion_quaternion.z() = msg_pt.orientation.z;
    aruco_pose_orientaion_quaternion.w() = msg_pt.orientation.w;
    aruco_pose_orientaion_quaternion.normalize();
    aruco_pose_orientaion_quaternion.inverse();

    target_pose_orientaion_quaternion = target_pose_orientaion_quaternion *
      aruco_pose_orientaion_quaternion.inverse();
    this->setPoseTarget(target_pose);
    trajectory_msgs::msg::JointTrajectoryPoint joint_goal;
    auto const [success, plan] = [this]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(this->plan(msg));
        return std::make_pair(ok, msg);
      }();
    if (success) {
      joint_goal = plan.trajectory_.joint_trajectory.points.back();
      this->publisher_joint_goal_->publish(joint_goal);
    }

    geometry_msgs::msg::PoseStamped pose_corrected;
    geometry_msgs::msg::PoseStamped pose_aruco;
    std_msgs::msg::Header a;
    a.frame_id = "/base_link";
    // a.stamp.sec=0;
    pose_aruco.header = a;
    pose_corrected.header = a;
    pose_corrected.pose = target_pose;
    // point_target.pose.orientation = target_pose_orientaion_quaternion;
    pose_aruco.pose = msg_pt;
    publisher_point_entry_rviz_->publish(pose_corrected);
    publisher_point_target_rviz_->publish(pose_aruco);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pose_correction", node_options);
  PoseCorrection move_group_interface(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
ros2 topic pub --once /introct/moveit/pose_goal geometry_msgs/msg/Pose
"{position: {x: -0.21042, y: -0.383917, z: -1.5128}, orientation:
{x: 0.689186, y: 0.484334, z: 0.440932, w: -0.309875}}"
*/
