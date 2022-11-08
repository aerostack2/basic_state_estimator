#include <as2_core/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "basic_state_estimator/plugins/state_estimator_plugin_ground_truth.hpp"
#include "basic_state_estimator/plugins/state_estimator_plugin_odom_only.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<as2::Node>("state_estimator");
  auto gt   = std::make_shared<state_estimation_plugin_ground_truth::Plugin>();
  auto odom = std::make_shared<state_estimation_plugin_odom_only::Plugin>();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
      std::make_shared<tf2_ros::TransformBroadcaster>(node);
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  gt->setup(node.get(), tf_buffer, tf_broadcaster, static_tf_broadcaster);
  odom->setup(node.get(), tf_buffer, tf_broadcaster, static_tf_broadcaster);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
