#include <as2_core/names/topics.hpp>
#include <as2_core/node.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <random>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include "basic_state_estimator/plugins/state_estimator_plugin_ground_truth.hpp"
#include "basic_state_estimator/plugins/state_estimator_plugin_mocap.hpp"
#include "basic_state_estimator/plugins/state_estimator_plugin_odom_only.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node  = std::make_shared<as2::Node>("mocap_state_estimator");
  auto mocap = std::make_shared<state_estimation_plugin_mocap::Plugin>();
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
      std::make_shared<tf2_ros::TransformBroadcaster>(node);
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  mocap->setup(node.get(), tf_buffer, tf_broadcaster, static_tf_broadcaster);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
