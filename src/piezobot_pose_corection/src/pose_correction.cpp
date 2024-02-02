#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

class PoseCorrection : public rclcpp::Node
{
public:
  PoseCorrection()
  : Node("PoseCorrection"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    link_6_frame_ = "link6";
    needle_holder_ = "needle_holder";
    base_link_ = "base_link";

    RCLCPP_INFO(
      this->get_logger(), "Transforming from %s to %s",
      needle_holder_.c_str(), link_6_frame_.c_str());

    publisher_corrected_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "servo_node/pose_target_cmds", 10);
    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(333),
      std::bind(&PoseCorrection::timer_callback, this));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_corrected_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform tf_attained;
  tf2::Transform tf_planned;
  geometry_msgs::msg::TransformStamped attained_previous_pose_;

  std::string link_6_frame_;
  std::string needle_holder_;
  std::string base_link_;
  void timer_callback()
  {
    try {
      auto attained_pose =
        tf_buffer_.lookupTransform(base_link_, needle_holder_, tf2::TimePointZero);
      if (attained_previous_pose_.transform != attained_pose.transform) {
        attained_previous_pose_ = attained_pose;
        auto planned_pose =
          tf_buffer_.lookupTransform(base_link_, link_6_frame_, tf2::TimePointZero);
        // double correction_translation_x = planned_pose.transform.translation.x - attained_pose.transform.translation.x;
        // double correction_translation_y = planned_pose.transform.translation.y - attained_pose.transform.translation.y;
        // double correction_translation_z = planned_pose.transform.translation.z - attained_pose.transform.translation.z;
        tf2::fromMsg(attained_pose.transform, tf_attained);
        tf2::fromMsg(planned_pose.transform, tf_planned);
        auto tf_error = tf_attained.inverseTimes(tf_planned);
        auto tf_plannned_corrected = tf_planned * tf_error;

        publisher_corrected_pose_->publish(tf_plannned_corrected);
      }
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "failed to get transform: %s", ex.what());
    }
  }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseCorrection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
