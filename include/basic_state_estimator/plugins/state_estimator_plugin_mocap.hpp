#ifndef __STATE_ESTIMATOR_PLUGIN_MOCAP_HPP__
#define __STATE_ESTIMATOR_PLUGIN_MOCAP_HPP__

#include <state_estimator_plugin_base.hpp>

namespace state_estimation_plugin_mocap {

class Plugin : public StateEstimatorPluginBase {
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_sub_;

public:
  Plugin() : StateEstimatorPluginBase(){};
  void on_setup() override {
    mocap_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos,
        std::bind(&Plugin::mocap_pose_callback, this, std::placeholders::_1));

    // publish static transform from earth to map and map to odom
    // TODO: MODIFY this to a initial earth to map transform (reading initial position from
    // parameters or msgs )
    geometry_msgs::msg::TransformStamped earth_to_map =
        as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
    geometry_msgs::msg::TransformStamped map_to_odom =
        as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);

    // TODO: CHECK IF WE NEED TO PUBLISH THIS PERIODICALLY
    publish_static_transform(earth_to_map);
    publish_static_transform(map_to_odom);
  };

private:
  void mocap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      // mocap could have a different frame_id, we will publish the transform from earth to
      // base_link without checking origin frame_id

      // TODO : IMPLEMENT THIS
  };
};

}  // namespace state_estimation_plugin_mocap

#endif  // __STATE_ESTIMATOR_PLUGIN_ODOM_ONLY_HPP__
