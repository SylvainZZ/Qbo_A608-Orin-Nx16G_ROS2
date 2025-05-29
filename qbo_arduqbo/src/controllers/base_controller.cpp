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
  if (driver_->getOdometry(x, y, th) < 0) return;

  float dx = (std::hypot(x - x_, y - y_)) / elapsed;
  float dth = (th - th_) / elapsed;

  x_ = x; y_ = y; th_ = th;

  geometry_msgs::msg::Quaternion q;
  tf2::Quaternion q_tf;
  q_tf.setRPY(0, 0, th_);
  q = tf2::toMsg(q_tf);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = dx;
  odom.twist.twist.angular.z = dth;
  odom_pub_->publish(odom);

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
