#ifndef __STATE_ESTIMATOR_PLUGIN_MOCAP_HPP__
#define __STATE_ESTIMATOR_PLUGIN_MOCAP_HPP__

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/duration.hpp>
#include <state_estimator_plugin_base.hpp>

namespace state_estimation_plugin_mocap {

class Plugin : public StateEstimatorPluginBase {
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_sub_;

  tf2::Transform earth_to_map_      = tf2::Transform::getIdentity();
  const tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();  // ALWAYS IDENTITY
  tf2::Transform odom_to_base_      = tf2::Transform::getIdentity();

  bool has_earth_to_map_ = false;

public:
  Plugin() : StateEstimatorPluginBase(){};
  void on_setup() override {
    mocap_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos,
        std::bind(&Plugin::mocap_pose_callback, this, std::placeholders::_1));

    // publish static transform from earth to map and map to odom
    // TODO: MODIFY this to a initial earth to map transform (reading initial position from
    // parameters or msgs )

    geometry_msgs::msg::TransformStamped map_to_odom =
        as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);
    publish_static_transform(map_to_odom);

    has_earth_to_map_ = false;
  };

  const geometry_msgs::msg::TwistStamped& twist_from_pose(
      const geometry_msgs::msg::PoseStamped& pose) {
    const double alpha = 0.1;

    const auto last_time = twist_msg_.header.stamp;
    auto dt              = (rclcpp::Time(pose.header.stamp) - last_time).seconds();

    if (dt <= 0) {
      RCLCPP_WARN(node_ptr_->get_logger(), "dt <= 0");
      return twist_msg_;
    }

    static tf2::Vector3 last_pose =
        tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    tf2::Vector3 current_pose =
        tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    tf2::Vector3 vel = (current_pose - last_pose) / dt;

    vel = alpha * vel + (1 - alpha) * tf2::Vector3(twist_msg_.twist.linear.x,
                                                   twist_msg_.twist.linear.y,
                                                   twist_msg_.twist.linear.z);

    twist_msg_.header.stamp   = pose.header.stamp;
    twist_msg_.twist.linear.x = vel.x();
    twist_msg_.twist.linear.y = vel.y();
    twist_msg_.twist.linear.z = vel.z();

    // TODO: add angular velocity -> this_could_be_obtained_from_imu
    twist_msg_.twist.angular.x = 0;
    twist_msg_.twist.angular.y = 0;
    twist_msg_.twist.angular.z = 0;

    return twist_msg_;
  };

  geometry_msgs::msg::TwistStamped twist_msg_;

private:
  void mocap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // mocap could have a different frame_id, we will publish the transform from earth to
    // base_link without checking origin frame_id

    if (!has_earth_to_map_) {
      earth_to_map_ = tf2::Transform(
          tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                          msg->pose.orientation.w),
          tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

      geometry_msgs::msg::TransformStamped earth_to_map;
      earth_to_map.transform       = tf2::toMsg(earth_to_map_);
      earth_to_map.header.stamp    = msg->header.stamp;
      earth_to_map.header.frame_id = get_earth_frame();
      earth_to_map.child_frame_id  = get_map_frame();
      publish_static_transform(earth_to_map);
      has_earth_to_map_ = true;
    }
    odom_to_base_ =
        map_to_odom_.inverse() * earth_to_map_.inverse() *
        tf2::Transform(
            tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                            msg->pose.orientation.z, msg->pose.orientation.w),
            tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

    geometry_msgs::msg::TransformStamped odom_to_base_msg;
    odom_to_base_msg.transform       = tf2::toMsg(odom_to_base_);
    odom_to_base_msg.header.stamp    = msg->header.stamp;
    odom_to_base_msg.header.frame_id = get_odom_frame();
    odom_to_base_msg.child_frame_id  = get_base_frame();
    publish_transform(odom_to_base_msg);

    // Publish pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = msg->header.stamp;
    pose_msg.header.frame_id = get_earth_frame();
    pose_msg.pose            = msg->pose;
    publish_pose(pose_msg);

    // Compute twist from mocap
    publish_twist(twist_from_pose(pose_msg));
  };
};

}  // namespace state_estimation_plugin_mocap

#endif  // __STATE_ESTIMATOR_PLUGIN_ODOM_ONLY_HPP__
