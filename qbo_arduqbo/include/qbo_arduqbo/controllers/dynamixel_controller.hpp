#ifndef DYNAMIXEL_CONTROLLER_HPP
#define DYNAMIXEL_CONTROLLER_HPP

#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <qbo_msgs/msg/motor_state.hpp>
#include <qbo_msgs/srv/torque_enable.hpp>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#define P_TORQUE_ENABLE           24
#define P_CW_COMPILANCE_SLOPE     28
#define P_CCW_COMPILANCE_SLOPE    29
#define P_GOAL_POSITION_L         30
#define P_GOAL_SPEED_L            32
#define P_TORQUE_LIMIT_L          34
#define P_PRESENT_POSITION_L      36
#define P_PRESENT_SPEED_L         38
#define P_PRESENT_LOAD_L          40
#define P_PRESENT_VOLTAGE         42
#define P_PRESENT_TEMPERATURE     43
#define P_MOVING                  46

inline double radians(double angle) {
    return angle * M_PI / 180.0;
}

class DynamixelServo
{
public:
    DynamixelServo(const std::shared_ptr<rclcpp::Node> & node,
                   const std::string & name,
                   dynamixel::PortHandler *portHandler,
                   dynamixel::PacketHandler *packetHandler);

    DynamixelServo(const std::shared_ptr<rclcpp::Node> & node,
                   const std::string & name,
                   const std::string & usb_port = "/dev/ttyUSB0",
                   int baud_rate = 57600,
                   double protocol_version = 1.0,
                   bool single = true);

    ~DynamixelServo();

    void setAngle(float ang, float velocity = 1.0f);
    float getAngle();
    void changeTorque(int torque);

    bool servoTorqueEnable(
        const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request> req,
        std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res);

    // void setParams(const rclcpp::ParameterMap & params);
    // void setParams();
    void setParams(const std::string & joint_name);

    int id_;
    bool invert_;
    float angle_;
    int neutral_;
    int ticks_;
    float max_angle_;
    float min_angle_;
    float rad_per_tick_;
    float max_speed_;
    float range_;
    std::string name_;
    std::string joint_name_;

protected:
    std::shared_ptr<rclcpp::Node> node_;
    dynamixel::PortHandler   *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    bool port_owner_;
    rclcpp::Publisher<qbo_msgs::msg::MotorState>::SharedPtr servo_state_pub_;
    rclcpp::Service<qbo_msgs::srv::TorqueEnable>::SharedPtr servo_torque_enable_srv_;
};

class DynamixelController
{
public:
    explicit DynamixelController(const std::shared_ptr<rclcpp::Node> & node);
    ~DynamixelController();

private:
    void publishJointStates();
    void publishMotorStates();
    void jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void cmdTimerCallback();

    std::shared_ptr<rclcpp::Node> node_;
    std::string usb_port_;
    int baud_rate_;
    double protocol_version_;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    std::vector<std::unique_ptr<DynamixelServo>> servos_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<qbo_msgs::msg::MotorState>::SharedPtr dynamixel_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;

    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr dynamixel_state_timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;
    sensor_msgs::msg::JointState joint_cmd_msg_;
    bool has_new_cmd_ = false;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    DynamixelWorkbench dxl_wb_;
    std::shared_ptr<diagnostic_updater::Updater> diagnostics_;


};

#endif  // DYNAMIXEL_CONTROLLER_HPP
