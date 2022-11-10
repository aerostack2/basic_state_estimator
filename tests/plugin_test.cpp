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

#define SPEED 0.50

class MocapMock : public as2::Node {
public:
  MocapMock() : as2::Node("mocap_mock") {
    mocap_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos);
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&MocapMock::timer_callback, this));

    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    as2::frame::eulerToQuaternion(0.0, 0.0, M_PI_2 / 2.0, msg.pose.orientation);
    // as2::frame::eulerToQuaternion(0.0, 0.0, 0.0, msg.pose.orientation);
  };

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped msg;

  double compute_dt() {
    static rclcpp::Time last_time = now();
    rclcpp::Time current_time     = now();
    double dt                     = (current_time - last_time).seconds();
    last_time                     = current_time;
    return dt;
  }

  double generate_noise() {
    static std::default_random_engine generator;
    static std::normal_distribution<double> distribution(0.0, 0.01);
    return distribution(generator);
  }

  void timer_callback() {
    auto dt             = compute_dt();
    msg.header.stamp    = now();
    msg.header.frame_id = "earth";
    msg.pose.position.x += SPEED * dt + generate_noise();
    /* msg.pose.position.y += SPEED * dt + generate_noise();
    msg.pose.position.z += SPEED * dt + generate_noise(); */
    /* msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0; */
    mocap_pub_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<as2::Node>("state_estimator");
  /* auto gt    = std::make_shared<state_estimation_plugin_ground_truth::Plugin>();
  auto odom  = std::make_shared<state_estimation_plugin_odom_only::Plugin>(); */
  auto mocap = std::make_shared<state_estimation_plugin_mocap::Plugin>();

  auto mock = std::make_shared<MocapMock>();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
      std::make_shared<tf2_ros::TransformBroadcaster>(node);
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  /* gt->setup(node.get(), tf_buffer, tf_broadcaster, static_tf_broadcaster);
  odom->setup(node.get(), tf_buffer, tf_broadcaster, static_tf_broadcaster); */
  mocap->setup(node.get(), tf_buffer, tf_broadcaster, static_tf_broadcaster);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(mock);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
