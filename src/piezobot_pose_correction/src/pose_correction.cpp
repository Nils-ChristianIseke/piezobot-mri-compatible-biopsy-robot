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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "rosgraph_msgs/msg/clock.hpp"


using moveit::planning_interface::MoveGroupInterface;

class PoseCorrection : public MoveGroupInterface
{
public:
  explicit PoseCorrection(rclcpp::Node::SharedPtr node)
  : MoveGroupInterface(node, "arm")
  {
   
    publisher_joint_goal_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "piezobot/moveit/joint_goal_fine", 10);

    pose_from_segmentation = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_from_segmentation", 10, [this](geometry_msgs::msg::PoseStamped msg_ft)
      {callback_pose_from_segmentation(msg_ft);});



  }

private:

  // geometry_msgs::msg::PoseStamped joint_group_positions = this->getCurrentPose();
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_joint_goal_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_from_segmentation;


  double millimeter_to_meter_ = 1000;

  void callback_pose_goal(geometry_msgs::msg::PoseStamped msg_pose)
  {
    this->setPoseTarget(msg_pose);
  }

  void callback_pose_from_segmentation(geometry_msgs::msg::PoseStamped msg_pt)
  {
    geometry_msgs::msg::PoseStamped target_pose = msg_pt;
    this->setPoseTarget(target_pose);
    trajectory_msgs::msg::JointTrajectoryPoint joint_goal;
    auto const [success, plan] = [this]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(this->plan(msg));
        return std::make_pair(ok, msg);
      }();
    if (success) {
      std_msgs::msg::Float64MultiArray joint_goal_msg;
      joint_goal_msg.data.resize(plan.trajectory_.joint_trajectory.points.back().positions.size());
      for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.back().positions.size(); ++i) {
        joint_goal_msg.data[i] = plan.trajectory_.joint_trajectory.points.back().positions[i];
      }
      this->publisher_joint_goal_->publish(joint_goal_msg);
    }
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
