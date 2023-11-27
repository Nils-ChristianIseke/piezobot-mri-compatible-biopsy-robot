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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/robot_state.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <chrono>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


using moveit::planning_interface::MoveGroupInterface;

class MoveGroup : public MoveGroupInterface
{
public:
  explicit MoveGroup(rclcpp::Node::SharedPtr node)
  : MoveGroupInterface(node, "arm")
  {
    publisher_joint_goal_ = node->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "introct/moveit/joint_goal_fine", 10);

    publisher_joint_differences_ = node->create_publisher<sensor_msgs::msg::JointState>(
      "introct/moveit/difference_to_goal_fine", 10);

    subscriber_point_entry_ = node->create_subscription<geometry_msgs::msg::Point>(
      "introct/navigation/point_entry", 10, [this](geometry_msgs::msg::Point needle_entry_point)
      {callback_point_entry(needle_entry_point);});

    publisher_point_entry_rviz_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "rviz/point_entry", 10);

    publisher_point_target_rviz_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "rviz/point_target", 10);

    subscriber_point_target_ = node->create_subscription<geometry_msgs::msg::Point>(
      "introct/navigation/point_target", 10, [this](geometry_msgs::msg::Point needle_target_point)
      {callback_point_target(needle_target_point);});

    subscriber_pose_goal_ = node->create_subscription<geometry_msgs::msg::Pose>(
      "introct/moveit/pose_goal", 10, [this](geometry_msgs::msg::Pose pose_goal)
      {callback_pose_goal(pose_goal);});
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_joint_goal_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_differences_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_point_entry_rviz_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_point_target_rviz_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    publisher_forward_position_controller_commands_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_pose_goal_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_point_entry_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_point_target_;

  double millimeter_to_meter_ = 1000;
  geometry_msgs::msg::Point needle_point_entry_;
  geometry_msgs::msg::Point needle_point_target_;

  rclcpp::Logger const logger_ = rclcpp::get_logger("Feinpositionierung_target");

  trajectory_msgs::msg::JointTrajectoryPoint joint_goal_;
  sensor_msgs::msg::JointState joint_states_target_;

  Eigen::Vector3d entry_point_vector_;
  Eigen::Vector3d target_point_vector_;
  Eigen::Vector3d target_vector_;
  geometry_msgs::msg::Point entry_point_;
  geometry_msgs::msg::Point target_point_;


  void callback_pose_goal(geometry_msgs::msg::Pose msg_pose)
  {
    this->setPoseTarget(msg_pose);
  }

  //   }

  /**
   * @brief Callback function of subscriber_point_target_.
   * Receives the target point, defined in Slicer and calculate the target pose of the roboters
   * Endeffector.
   *
   * @param msg_pt Target point, defined in Slicer
   */
  void callback_point_target(geometry_msgs::msg::Point msg_pt)
  {
    moveit_msgs::msg::Constraints joint_constraints;
    // moveit_msgs::msg::JointConstraint joint_5_constraint;
    // joint_5_constraint.joint_name = "Joint_5";
    // joint_5_constraint.position = 0.025;
    // joint_5_constraint.tolerance_above = 0.010;
    // joint_5_constraint.tolerance_below = 0.010;
    // joint_5_constraint.weight = 0.5;
    // // joint_constraints.joint_constraints.push_back(joint_5_constraint);
    // moveit_msgs::msg::JointConstraint joint_6_constraint;
    // joint_6_constraint.joint_name = "Joint_6";
    // joint_6_constraint.position = -0.022;
    // joint_6_constraint.tolerance_above = 0.1;
    // joint_6_constraint.tolerance_below = 0.1;
    // joint_6_constraint.weight = 0.0;
    // joint_constraints.joint_constraints.push_back(joint_6_constraint);
    // moveit_msgs::msg::JointConstraint joint_10_constraint;
    // joint_10_constraint.joint_name = "Joint_10";
    // joint_10_constraint.position = -0.05;
    // joint_10_constraint.tolerance_above = 0.1;
    // joint_10_constraint.tolerance_below = 0.1;
    // joint_10_constraint.weight = 0.0;
    // joint_constraints.joint_constraints.push_back(joint_10_constraint);

    // this->setPathConstraints(joint_constraints);
    target_point_.x = msg_pt.x / millimeter_to_meter_;
    target_point_.y = msg_pt.y / millimeter_to_meter_;
    target_point_.z = msg_pt.z / millimeter_to_meter_;
    target_point_vector_ = Eigen::Vector3d(target_point_.x, target_point_.y, target_point_.z);
    target_vector_ = target_point_vector_ - entry_point_vector_;
    geometry_msgs::msg::PoseStamped target_pose = this->getPoseTarget();

    RCLCPP_INFO_STREAM(
      this->logger_, "Target pose is: "
        << "position(x: " << target_pose.pose.position.x
        << ", y: " << target_pose.pose.position.y
        << ", z: " << target_pose.pose.position.z << "), "
        << "orientation(x: " << target_pose.pose.orientation.x
        << ", y: " << target_pose.pose.orientation.y
        << ", z: " << target_pose.pose.orientation.z
        << ", w: " << target_pose.pose.orientation.w << ")");

    auto const [success, plan] = [this]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(this->plan(msg));
        return std::make_pair(ok, msg);
      }();
    if (success) {
      joint_goal_ = plan.trajectory_.joint_trajectory.points.back();
      // this->publisher_joint_goal_->publish(joint_goal_);
      std_msgs::msg::Float64MultiArray controller_command;
      controller_command.set__data(joint_goal_.positions);
      // controller_command.set__layout();
      this->publisher_forward_position_controller_commands_->publish(controller_command);
    }

    geometry_msgs::msg::PointStamped point_entry;
    geometry_msgs::msg::PointStamped point_target;
    std_msgs::msg::Header a;
    a.frame_id = "/base_link";
    point_target.header = a;
    point_entry.header = a;
    point_entry.point = entry_point_;
    point_target.point = target_point_;

    publisher_point_entry_rviz_->publish(point_entry);
    publisher_point_target_rviz_->publish(point_target);
  }

  /**
   * @brief Callback function of subsriber_point_entry.
   * Receives the entry point, defined in Slicer and update the entry_point_ of the instance.
   *
   * @param msg_pt Entry point, defined in Slicer.
   */
  void callback_point_entry(geometry_msgs::msg::Point msg_pt)
  {
    needle_point_entry_.x = msg_pt.x / millimeter_to_meter_;
    needle_point_entry_.y = msg_pt.y / millimeter_to_meter_;
    needle_point_entry_.z = msg_pt.z / millimeter_to_meter_;
    entry_point_vector_ = Eigen::Vector3d(
      needle_point_entry_.x, needle_point_entry_.y,
      needle_point_entry_.z);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("target_fine", node_options);
  MoveGroup move_group_interface(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
ros2 topic pub --once /introct/moveit/pose_goal geometry_msgs/msg/Pose
 "{position: {x: -0.21042, y: -0.383917, z: -1.5128}, orientation:
 {x: 0.689186, y: 0.484334, z: 0.440932, w: -0.309875}}"

 */
