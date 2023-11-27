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

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/robot_state.h>
#include <math.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sstream>
#include <functional>
#include <memory>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/geometry_msgs/msg/point.hpp>
#include <geometry_msgs/geometry_msgs/msg/point_stamped.hpp>


using moveit::planning_interface::MoveGroupInterface;

class MoveGroup : public MoveGroupInterface
{
public:
  explicit MoveGroup(rclcpp::Node::SharedPtr node)
  : MoveGroupInterface(node, "introct_arm")
  {
    publisher_joint_goal_ = node->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "introct/moveit/joint_goal", 10);

    publisher_joint_differences_ = node->create_publisher<sensor_msgs::msg::JointState>(
      "introct/moveit/difference_to_goal", 10);

    publisher_target_pose_ = node->create_publisher<geometry_msgs::msg::Pose>(
      "introct/moveit/pose_goal", 10);

    publisher_target_pose_stamped_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "introct/moveit/pose_goal_stamped", 10);

    publisher_point_entry_rviz_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "rviz/point_entry", 10);

    publisher_point_target_rviz_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "rviz/point_target", 10);

    subscriber_point_entry_ = node->create_subscription<geometry_msgs::msg::Point>(
      "introct/navigation/point_entry", 10, [this](geometry_msgs::msg::Point message_point)
      {callback_point_entry(message_point);});

    subscriber_point_target_ = node->create_subscription<geometry_msgs::msg::Point>(
      "introct/navigation/point_target", 10, [this](geometry_msgs::msg::Point message_point)
      {callback_point_target(message_point);});

    subscriber_joint_states_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "introct/manipulator/joint_states", 10,
      [this](sensor_msgs::msg::JointState msg_joint_states)
      {callback_joint_states(msg_joint_states);});
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_joint_goal_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_differences_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_target_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_target_pose_stamped_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_point_entry_rviz_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_point_target_rviz_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_pose_goal_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_joint_states_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_point_entry_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_point_target_;

  double millimeter_to_meter_ = 1000;
  geometry_msgs::msg::Point entry_point_;
  geometry_msgs::msg::Point needle_point_target_;
  trajectory_msgs::msg::JointTrajectoryPoint joint_goal_;
  sensor_msgs::msg::JointState joint_states_target_;
  Eigen::Vector3d entry_point_vector_;
  Eigen::Vector3d target_point_vector_;
  Eigen::Vector3d target_vector_;

  rclcpp::Logger const logger_ = rclcpp::get_logger("Grobpositionierung_target");

  /**
   * @brief Callback function of subscriber_joint_states_.
   *  Receives joint states of introct, and calculates the joint difference between the current
   * joint values and ther target joint values of the manual axes. Publishs the information with
   * the publisher publisher_joint_differences_.
   *
   * @param joint_states_introct The values of the introct joints
   */
  void callback_joint_states(sensor_msgs::msg::JointState joint_states_introct)
  {
    int i = 0;
    std::vector<double, std::allocator<double>> joint_differences;
    sensor_msgs::msg::JointState msg_joint_differences;
    for (double joint_angle : this->joint_goal_.positions) {
      joint_differences.insert(
        joint_differences.begin() + i, joint_angle -
        joint_states_introct.position[i]);
      i++;
      // only need the joint differences of manual joints
      if (i > 3) {
        break;
      }
    }

    msg_joint_differences.set__position(joint_differences);
    this->publisher_joint_differences_->publish(msg_joint_differences);
  }

  /**
   * @brief Callback function of subscriber_point_target_.
   * Receives the target point, defined in Slicer and calculate the target pose of the roboters
   * Endeffector.
   *
   * @param msg_pt Target point, defined in Slicer
   */
  void callback_point_target(geometry_msgs::msg::Point msg_pt)
  {
    needle_point_target_.x = msg_pt.x / millimeter_to_meter_;
    needle_point_target_.y = msg_pt.y / millimeter_to_meter_;
    needle_point_target_.z = msg_pt.z / millimeter_to_meter_;
    target_point_vector_ = Eigen::Vector3d(
      needle_point_target_.x, needle_point_target_.y,
      needle_point_target_.z);
  }

  /**
   * @brief Callback function of subsriber_point_entry.
   * Receives the entry point, defined in Slicer and update the entry_point_ of the instance.
   *
   * @param msg_pt Entry point, defined in Slicer.
   */

  void callback_point_entry(geometry_msgs::msg::Point msg_pt)
  {
    // convert from millimeter to meter, cause slicer uses millimter and ros meter
    entry_point_.x = msg_pt.x / millimeter_to_meter_;
    entry_point_.y = msg_pt.y / millimeter_to_meter_;
    entry_point_.z = msg_pt.z / millimeter_to_meter_;

    // Calculate the target vector to get the target path of the needle
    entry_point_vector_ = Eigen::Vector3d(entry_point_.x, entry_point_.y, entry_point_.z);
    target_vector_ = target_point_vector_ - entry_point_vector_;

    // Needle_feed is just needed to know how much the needle needs to be moved
    double needle_feed = target_vector_.norm();


    // use eigen to calculate the target_quaternion (Pose) of the needle
    Eigen::Vector3d unit_vector_x = Eigen::Vector3d::UnitX();
    Eigen::Quaterniond target_quaternion = Eigen::Quaterniond::FromTwoVectors(
      unit_vector_x,
      target_vector_);


    geometry_msgs::msg::Pose target_pose;
    target_pose.position = entry_point_;
    target_pose.orientation.x = target_quaternion.x();
    target_pose.orientation.y = target_quaternion.y();
    target_pose.orientation.z = target_quaternion.z();
    target_pose.orientation.w = target_quaternion.w();

    this->setPoseTarget(target_pose);
    RCLCPP_INFO_STREAM(
      this->logger_, "Target pose is: "
        << "position(x: "
        << target_pose.position.x << ", y: "
        << target_pose.position.y << ", z: "
        << target_pose.position.z << "), "
        << "orientation(x: " << target_pose.orientation.x
        << ", y: " << target_pose.orientation.y << ", z: "
        << target_pose.orientation.z << ", w: "
        << target_pose.orientation.w << ")");

    // ============= Uncomment to set constraints =============

    // moveit_msgs::msg::Constraints joint_constraints;
    // moveit_msgs::msg::JointConstraint joint_5_constraint;
    // joint_5_constraint.joint_name = "Joint_5";
    // joint_5_constraint.position = 0.025;
    // joint_5_constraint.tolerance_above = 0.010;
    // joint_5_constraint.tolerance_below = 0.010;
    // joint_5_constraint.weight = 1;
    // joint_constraints.joint_constraints.push_back(joint_5_constraint);

    // moveit_msgs::msg::JointConstraint joint_6_constraint;
    // joint_6_constraint.joint_name = "Joint_6";
    // joint_6_constraint.position = -0.022;
    // joint_6_constraint.tolerance_above = 0.008;
    // joint_6_constraint.tolerance_below = 0.008;
    // joint_6_constraint.weight = 1.0;
    // joint_constraints.joint_constraints.push_back(joint_6_constraint);

    // moveit_msgs::msg::JointConstraint joint_10_constraint;
    // joint_10_constraint.joint_name = "Joint_10";
    // joint_10_constraint.position = -0.02;
    // joint_10_constraint.tolerance_above = 0.01;
    // joint_10_constraint.tolerance_below = 0.01;
    // joint_10_constraint.weight = 1.0;
    // joint_constraints.joint_constraints.push_back(joint_10_constraint);

    // moveit_msgs::msg::JointConstraint joint_3_constraint;
    // joint_3_constraint.joint_name = "Joint_3";
    // joint_3_constraint.position = -0.1 + 0.14;
    // joint_3_constraint.tolerance_above = 0.05;
    // joint_3_constraint.tolerance_below = 0.05;
    // joint_3_constraint.weight = 1.0;
    // joint_constraints.joint_constraints.push_back(joint_3_constraint);

    // this->setPathConstraints(joint_constraints);

    //========================= uncomment end ===========================================


    // Calculate the motion plan. Maybe it is worth trying to just use IK especially for the rough
    // positioning, cause there is no need for a motion plan
    auto const [success, plan] = [this]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(this->plan(msg));
        return std::make_pair(ok, msg);
      }();
    if (success) {
      joint_goal_ = plan.trajectory_.joint_trajectory.points.back();
      this->publisher_joint_goal_->publish(joint_goal_);
    }

    publisher_target_pose_->publish(target_pose);

    // Publish point_entry and point_target as stamped pose, so that they are visible in rviz

    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.frame_id = "/base_link";
    target_pose_stamped.pose = target_pose;
    publisher_target_pose_stamped_->publish(target_pose_stamped);
    geometry_msgs::msg::PointStamped point_entry;
    geometry_msgs::msg::PointStamped point_target;
    std_msgs::msg::Header a;
    a.frame_id = "/base_link";
    point_target.header = a;
    point_entry.header = a;
    point_entry.point = entry_point_;
    point_target.point = needle_point_target_;

    publisher_point_entry_rviz_->publish(point_entry);
    publisher_point_target_rviz_->publish(point_target);

    // Print out who much the needle needs to be proceeded to reach the target point.
    RCLCPP_INFO(this->logger_, "Needlefeed: %f mm", needle_feed * 1000);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("target_publisher", node_options);
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
