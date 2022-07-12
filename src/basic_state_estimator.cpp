#include "basic_state_estimator.hpp"

BasicStateEstimator::BasicStateEstimator() : as2::Node("basic_state_estimator") {
  this->declare_parameter<bool>("odom_only", false);
  this->declare_parameter<bool>("ground_truth", false);
  this->declare_parameter<bool>("sensor_fusion", false);
}

void BasicStateEstimator::run() {
  // TODO: SENSOR FUSION
  geometry_msgs::msg::Transform map2odom_tf;
  map2odom_tf = calculateLocalization();
  updateOdomTfDrift(odom2baselink_tf_.transform, map2odom_tf);
  publishTfs();
  getGlobalRefState();
  publishStateEstimation();
}

void BasicStateEstimator::setupNode() {
  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->generate_global_name(as2_names::topics::sensor_measurements::odom),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&BasicStateEstimator::odomCallback, this, std::placeholders::_1));
  
  gt_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->generate_global_name(as2_names::topics::ground_truth::pose),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&BasicStateEstimator::gtPoseCallback, this, std::placeholders::_1));
  
  gt_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->generate_global_name(as2_names::topics::ground_truth::twist),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&BasicStateEstimator::gtTwistCallback, this, std::placeholders::_1));

  pose_estimated_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos);
  twist_estimated_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos);
  // pose_estimated_pub_ =
  // this->create_publisher<geometry_msgs::msg::PoseStamped>(generate_global_name("self_localization/pose"),
  // as2_names::topics::self_localization::qos); twist_estimated_pub_ =
  // this->create_publisher<geometry_msgs::msg::TwistStamped>(generate_global_name("self_localization/twist"),
  // as2_names::topics::self_localization::qos);
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_estimated_pub_;
}

void BasicStateEstimator::setupTfTree() {
  this->get_parameter("odom_only", odom_only_);
  this->get_parameter("ground_truth", ground_truth_);
  this->get_parameter("sensor_fusion", sensor_fusion_);

  if (odom_only_) {
    RCLCPP_INFO(get_logger(), "ODOM ONLY MODE");
  }

  if (ground_truth_) {
    RCLCPP_INFO(get_logger(), "GROUND TRUTH MODE");
  }

  if (sensor_fusion_) {
    RCLCPP_INFO(get_logger(), "SENSOR FUSION MODE");
  }

  tf2_fix_transforms_.clear();
  // global reference to drone reference
  std::string ns = this->get_namespace();
  global_ref_frame_ = "earth";
  map_frame_ = generateTfName(ns, "map");
  odom_frame_ = generateTfName(ns, "odom");
  baselink_frame_ = generateTfName(ns, "base_link");

  getStartingPose(global_ref_frame_, map_frame_);

  // init map_2_odom
  map2odom_tf_.header.frame_id = map_frame_;
  map2odom_tf_.child_frame_id = odom_frame_;
  map2odom_tf_.transform.rotation.w = 1.0f;

  // init odom_2_baselink
  odom2baselink_tf_.header.frame_id = odom_frame_;
  odom2baselink_tf_.child_frame_id = baselink_frame_;
  odom2baselink_tf_.transform.rotation.w = 1.0f;

  RCLCPP_INFO(get_logger(), "%s -> %s", global_ref_frame_.c_str(), map_frame_.c_str());
  RCLCPP_INFO(get_logger(), "%s -> %s", map2odom_tf_.header.frame_id.c_str(),
              map2odom_tf_.child_frame_id.c_str());
  RCLCPP_INFO(get_logger(), "%s -> %s", odom2baselink_tf_.header.frame_id.c_str(),
              odom2baselink_tf_.child_frame_id.c_str());

  // init Tf tree
  publishTfs();
}

void BasicStateEstimator::getStartingPose(const std::string &_global_frame,
                                          const std::string &_map) {
  // TODO: Get starting pose

  // Default
  tf2_fix_transforms_.emplace_back(getTransformation(_global_frame, _map, 0, 0, 0, 0, 0, 0));
}

void BasicStateEstimator::updateOdomTfDrift(const geometry_msgs::msg::Transform _odom2baselink,
                                            const geometry_msgs::msg::Transform _map2baselink) {
  map2odom_tf_.transform.translation.x = _map2baselink.translation.x - _odom2baselink.translation.x;
  map2odom_tf_.transform.translation.y = _map2baselink.translation.y - _odom2baselink.translation.y;
  map2odom_tf_.transform.translation.z = _map2baselink.translation.z - _odom2baselink.translation.z;

  // Calculate relative orientation
  tf2::Quaternion map2baselink_orientation(_map2baselink.rotation.x, _map2baselink.rotation.y,
                                           _map2baselink.rotation.z, _map2baselink.rotation.w);
  tf2::Quaternion odom2baselink_orientation(_odom2baselink.rotation.x, _odom2baselink.rotation.y,
                                            _odom2baselink.rotation.z, _odom2baselink.rotation.w);
  tf2::Quaternion map2odom_orientation =
      map2baselink_orientation * tf2::inverse(odom2baselink_orientation);

  map2odom_tf_.transform.rotation.x = map2odom_orientation.x();
  map2odom_tf_.transform.rotation.y = map2odom_orientation.y();
  map2odom_tf_.transform.rotation.z = map2odom_orientation.z();
  map2odom_tf_.transform.rotation.w = map2odom_orientation.w();
}

geometry_msgs::msg::Transform BasicStateEstimator::calculateLocalization() {
  geometry_msgs::msg::Transform map2baselink;
  if (odom_only_) {
    map2baselink = odom2baselink_tf_.transform;
  }
  if (ground_truth_) {
    map2baselink.translation.x = gt_pose_.position.x;
    map2baselink.translation.y = gt_pose_.position.x;
    map2baselink.translation.z = gt_pose_.position.x;
    map2baselink.rotation.x = gt_pose_.orientation.x;
    map2baselink.rotation.y = gt_pose_.orientation.y;
    map2baselink.rotation.z = gt_pose_.orientation.z;
    map2baselink.rotation.w = gt_pose_.orientation.w;
  }
  if (!sensor_fusion_) {
    // TODO: SENSOR FUSION
  }
  return map2baselink;
}

void BasicStateEstimator::getGlobalRefState() {
  try {
    auto pose_transform =
        tf_buffer_->lookupTransform(global_ref_frame_, baselink_frame_, tf2::TimePointZero);
    global_ref_pose.position.x = pose_transform.transform.translation.x;
    global_ref_pose.position.y = pose_transform.transform.translation.y;
    global_ref_pose.position.z = pose_transform.transform.translation.z;
    global_ref_pose.orientation.x = pose_transform.transform.rotation.x;
    global_ref_pose.orientation.y = pose_transform.transform.rotation.y;
    global_ref_pose.orientation.z = pose_transform.transform.rotation.z;
    global_ref_pose.orientation.w = pose_transform.transform.rotation.w;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform Failure: %s\n",
                ex.what());  // Print exception which was caught
  }

  if (odom_only_) {
    global_ref_twist.header.frame_id = odom_frame_;
    global_ref_twist.twist.angular = odom_twist_.twist.angular;
    tf2::Quaternion orientation(
        odom2baselink_tf_.transform.rotation.x, odom2baselink_tf_.transform.rotation.y,
        odom2baselink_tf_.transform.rotation.z, odom2baselink_tf_.transform.rotation.w);
        Eigen::Vector3d odom_linear_twist(odom_twist_.twist.linear.x, odom_twist_.twist.linear.y,
                                          odom_twist_.twist.linear.z);
    Eigen::Vector3d global_linear_twist =
        as2::FrameUtils::convertFLUtoENU(orientation, odom_linear_twist); 
    global_ref_twist.twist.linear.x = global_linear_twist.x();
    global_ref_twist.twist.linear.y = global_linear_twist.y();
    global_ref_twist.twist.linear.z = global_linear_twist.z();
  }

  if (ground_truth_) {
    global_ref_twist = gt_twist_;
  }

  if (sensor_fusion_) {  // TODO: Sensor fusion
  }
}

// PUBLISH //

void BasicStateEstimator::publishTfs() {
  rclcpp::Time timestamp = this->get_clock()->now();
  for (geometry_msgs::msg::TransformStamped &transform : tf2_fix_transforms_) {
    transform.header.stamp = timestamp;
    tfstatic_broadcaster_->sendTransform(transform);
  }
  map2odom_tf_.header.stamp = timestamp;
  tf_broadcaster_->sendTransform(map2odom_tf_);
  odom2baselink_tf_.header.stamp = timestamp;
  tf_broadcaster_->sendTransform(odom2baselink_tf_);
}

void BasicStateEstimator::publishStateEstimation() {
  rclcpp::Time timestamp = this->get_clock()->now();
  pose_estimated_pub_->publish(generatePoseStampedMsg(timestamp));
  twist_estimated_pub_->publish(generateTwistStampedMsg(timestamp));
}

geometry_msgs::msg::PoseStamped BasicStateEstimator::generatePoseStampedMsg(
    const rclcpp::Time &_timestamp) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = _timestamp;
  pose_stamped.header.frame_id = global_ref_frame_;
  pose_stamped.pose.position = global_ref_pose.position;
  pose_stamped.pose.orientation = global_ref_pose.orientation;
  return pose_stamped;
}

geometry_msgs::msg::TwistStamped BasicStateEstimator::generateTwistStampedMsg(
    const rclcpp::Time &_timestamp) {
  geometry_msgs::msg::TwistStamped twist_stamped;
  twist_stamped.header.stamp = _timestamp;
  twist_stamped.header.frame_id = global_ref_twist.header.frame_id;  // TODO:Review ref frame
  twist_stamped.twist.linear = global_ref_twist.twist.linear;
  twist_stamped.twist.angular = global_ref_twist.twist.angular;
  return twist_stamped;
}

// CALLBACKS //

void BasicStateEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr _msg) {
  // rclcpp::Time timestamp = this->get_clock()->now();
  // odom2baselink_tf_.header.stamp = timestamp;
  odom2baselink_tf_.transform.translation.x = _msg->pose.pose.position.x;
  odom2baselink_tf_.transform.translation.y = _msg->pose.pose.position.y;
  odom2baselink_tf_.transform.translation.z = _msg->pose.pose.position.z;
  odom2baselink_tf_.transform.rotation.x = _msg->pose.pose.orientation.x;
  odom2baselink_tf_.transform.rotation.y = _msg->pose.pose.orientation.y;
  odom2baselink_tf_.transform.rotation.z = _msg->pose.pose.orientation.z;
  odom2baselink_tf_.transform.rotation.w = _msg->pose.pose.orientation.w;

  odom_twist_.header.frame_id = odom_frame_;
  odom_twist_.twist.linear = _msg->twist.twist.linear;
  odom_twist_.twist.angular = _msg->twist.twist.angular;
}

void BasicStateEstimator::gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg) {
  gt_pose_ = _msg->pose;
}

void BasicStateEstimator::gtTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr _msg) {
  gt_twist_.header.frame_id = _msg->header.frame_id;
  gt_twist_.twist = _msg->twist;
}

void BasicStateEstimator::cleanupNode(){
    // TODO: CLeanup Node
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn BasicStateEstimator::on_configure(const rclcpp_lifecycle::State &_state) {
  // Set subscriptions, publishers, services, actions, etc. here.
  setupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn BasicStateEstimator::on_activate(const rclcpp_lifecycle::State &_state) {
  // Set parameters?
  setupTfTree();

  return CallbackReturn::SUCCESS;
};

CallbackReturn BasicStateEstimator::on_deactivate(const rclcpp_lifecycle::State &_state) {
  // Clean up subscriptions, publishers, services, actions, etc. here.
  cleanupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn BasicStateEstimator::on_shutdown(const rclcpp_lifecycle::State &_state) {
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
};
