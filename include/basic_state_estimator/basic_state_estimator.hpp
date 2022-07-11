#ifndef BASIC_STATE_ESTIMATOR_HPP_
#define BASIC_STATE_ESTIMATOR_HPP_

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <rclcpp/rclcpp.hpp>

#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/tf_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#define ONLY_ODOM true
#define GROUND_TRUTH false

class BasicStateEstimator : public as2::Node
{
public:
  BasicStateEstimator();

  void setupNode();
  void cleanupNode();
  void setupTfTree();
  void run();
  void getStartingPose(const std::string &_earth_frame, const std::string &_map);
  void updateOdomTfDrift(const geometry_msgs::msg::Transform _odom2baselink,
                         const geometry_msgs::msg::Transform _map2baselink);
  geometry_msgs::msg::Transform calculateLocalization();
  void publishTfs();

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gt_twist_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_estimated_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_estimated_pub_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr _msg);
  void gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void gtTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr _msg);

  std::vector<geometry_msgs::msg::TransformStamped> tf2_fix_transforms_;
  geometry_msgs::msg::TransformStamped map2odom_tf_;
  geometry_msgs::msg::TransformStamped odom2baselink_tf_;
  geometry_msgs::msg::TwistStamped odom_twist_;
  geometry_msgs::msg::Pose gt_pose_;
  geometry_msgs::msg::TwistStamped gt_twist_;
  geometry_msgs::msg::Pose global_ref_pose;
  geometry_msgs::msg::TwistStamped global_ref_twist; // TODO:Review

  bool odom_only_;
  bool ground_truth_;
  bool sensor_fusion_;

  void getGlobalRefState();

  std::string global_ref_frame_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string baselink_frame_;

  void publishStateEstimation();
  geometry_msgs::msg::PoseStamped generatePoseStampedMsg(const rclcpp::Time &_timestamp);
  geometry_msgs::msg::TwistStamped generateTwistStampedMsg(const rclcpp::Time &_timestamp);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
};

#endif // BASIC_STATE_ESTIMATOR_HPP_
