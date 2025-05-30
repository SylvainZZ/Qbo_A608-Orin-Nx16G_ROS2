// base_controller_ros2.cpp
#include "qbo_arduqbo/controllers/base_controller.hpp"

BaseController::BaseController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("base", options), driver_(driver), broadcast_tf_(true), base_stop_(false), v_linear_(0.0), v_angular_(0.0), x_(0.0), y_(0.0), th_(0.0), v_dirty_(true)  {

  // (void)name;
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  declare_parameter("rate", 15.0);
  declare_parameter("topic", "/cmd_vel");
  declare_parameter("odom_topic", "/odom");
  declare_parameter("tf_odom_broadcast", true);

  get_parameter("rate", rate_);
  get_parameter("topic", cmd_topic_);
  get_parameter("odom_topic", odom_topic_);
  get_parameter("tf_odom_broadcast", broadcast_tf_);

  twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic_, 10, std::bind(&BaseController::twistCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  reset_stall_srv_ = create_service<std_srvs::srv::Empty>(
    "unlock_motors_stall", std::bind(&BaseController::resetStall, this, std::placeholders::_1, std::placeholders::_2));

  stop_base_srv_ = create_service<std_srvs::srv::Empty>(
    "stop_base", std::bind(&BaseController::stopBase, this, std::placeholders::_1, std::placeholders::_2));

  timer_ = create_wall_timer(std::chrono::milliseconds((int)(1000.0 / rate_)), std::bind(&BaseController::timerCallback, this));

  publishStaticTF();
  last_time_ = now();

    // Initialiser odometry message
  last_time_ = now();

  odom_.header.stamp = last_time_;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";

  // Position initiale
  odom_.pose.pose.position.x = 0.0;
  odom_.pose.pose.position.y = 0.0;
  odom_.pose.pose.position.z = 0.0;

  geometry_msgs::msg::Quaternion q = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
  odom_.pose.pose.orientation = q;

  // Covariance de la position
  odom_.pose.covariance = {
    0.0015, 0, 0, 0, 0, 0,
    0, 0.0015, 0, 0, 0, 0,
    0, 0, 1e-6, 0, 0, 0,
    0, 0, 0, 1e-6, 0, 0,
    0, 0, 0, 0, 1e-6, 0,
    0, 0, 0, 0, 0, 0.05
  };

  // Covariance de la vitesse
  odom_.twist.covariance = {
    0.0015, 0, 0, 0, 0, 0,
    0, 1e-6, 0, 0, 0, 0,
    0, 0, 1e-6, 0, 0, 0,
    0, 0, 0, 1e-6, 0, 0,
    0, 0, 0, 0, 1e-6, 0,
    0, 0, 0, 0, 0, 0.05
  };

  // Static TF base_footprint → base_link
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.stamp = now();
  static_tf.header.frame_id = "base_footprint";
  static_tf.child_frame_id = "base_link";
  static_tf.transform.translation.x = 0.0;
  static_tf.transform.translation.y = 0.0;
  static_tf.transform.translation.z = 0.02;

  tf2::Quaternion q_static;
  q_static.setRPY(0, 0, 0);
  static_tf.transform.rotation = tf2::toMsg(q_static);

  static_tf_broadcaster_->sendTransform(static_tf);

  RCLCPP_INFO(this->get_logger(), "✅ CBaseController initialized");
  // std::cout << "Nom du contrôleur: " << this->get_name() << std::endl;
  RCLCPP_INFO(this->get_logger(), "Rate loaded for CBaseController: %.2f Hz", rate_);

}

void BaseController::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  v_linear_ = (float)msg->linear.x;
  v_angular_ = (float)msg->angular.z;
  v_dirty_ = true;
}

void BaseController::timerCallback() {
  rclcpp::Time now = this->now();
  double elapsed = (now - last_time_).seconds();
  last_time_ = now;

  float x, y, th;
  int code = driver_->getOdometry(x, y, th);
  if (code < 0) {
      RCLCPP_ERROR(get_logger(), "Unable to get odometry from the base controller board, code: %d", code);
      return;
  } else {
      RCLCPP_DEBUG(get_logger(), "Odometry message from base controller board: x=%.2f, y=%.2f, th=%.2f", x, y, th);
  }
  // if (driver_->getOdometry(x, y, th) < 0) return;

  float dx = (std::hypot(x - x_, y - y_)) / elapsed;
  float dth = (th - th_) / elapsed;

  x_ = x; y_ = y; th_ = th;

  geometry_msgs::msg::Quaternion q;
  tf2::Quaternion q_tf;
  q_tf.setRPY(0, 0, th_);
  q = tf2::toMsg(q_tf);

  // nav_msgs::msg::Odometry odom;
  odom_.header.stamp = now;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";
  odom_.pose.pose.position.x = x_;
  odom_.pose.pose.position.y = y_;
  odom_.pose.pose.orientation = q;
  odom_.twist.twist.linear.x = dx;
  odom_.twist.twist.angular.z = dth;
  odom_pub_->publish(odom_);

  if (broadcast_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = q;
    tf_broadcaster_->sendTransform(tf);
  }

  if (v_dirty_) {
    if (base_stop_) {
      driver_->setSpeed(0, 0);
      base_stop_ = false;
    } else {
      driver_->setSpeed(v_linear_, v_angular_);
    }
    v_dirty_ = false;
  }
}

void BaseController::publishStaticTF() {
  if (static_tf_sent_) return;

  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.stamp = now();
  static_tf.header.frame_id = "base_footprint";
  static_tf.child_frame_id = "base_link";
  static_tf.transform.translation.z = 0.02;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  static_tf.transform.rotation = tf2::toMsg(q);
  static_tf_broadcaster_->sendTransform(static_tf);
  static_tf_sent_ = true;
}

bool BaseController::resetStall(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
  return driver_->resetStall() >= 0;
}

bool BaseController::stopBase(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                              std::shared_ptr<std_srvs::srv::Empty::Response>) {
  base_stop_ = true;
  return true;
}

bool BaseController::setOdometry(const std::shared_ptr<qbo_msgs::srv::setOdometry::Request>,
                                std::shared_ptr<qbo_msgs::srv::setOdometry::Response>) {

}