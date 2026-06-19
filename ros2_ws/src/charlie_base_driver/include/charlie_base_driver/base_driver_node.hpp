#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "charlie_base_driver/serial_port.hpp"

namespace charlie_base_driver {

class BaseDriverNode : public rclcpp::Node {
public:
    BaseDriverNode();

private:
    void declare_parameters();
    void load_parameters();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void control_update();

    void update_target_wheel_speeds();
    void apply_cmd_vel_timeout();

    void open_serial();
    void send_speed_command();
    void read_serial();

    std::string make_speed_command_string() const;
    void handle_teensy_line(const std::string & line);

    void process_odom_packet(
        double left_total_m,
        double right_total_m,
        double left_speed_mps,
        double right_speed_mps,
        int status
    );

    void publish_odom(
        const rclcpp::Time & stamp,
        double linear_velocity_mps,
        double angular_velocity_radps
    );

    void normalize_theta();

    // Parameters
    std::string serial_port_name_;
    int baud_rate_;
    double wheel_separation_;
    double cmd_timeout_s_;
    double command_rate_hz_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;

    // Latest robot-level command
    double cmd_linear_x_;
    double cmd_angular_z_;

    // Wheel-level command
    double left_target_mps_;
    double right_target_mps_;

    // Robot pose
    double x_;
    double y_;
    double theta_;

    // Wheel odom state
    bool received_first_odom_;
    double previous_left_total_m_;
    double previous_right_total_m_;
    rclcpp::Time last_odom_time_;
    int last_status_;

    // Timing
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_debug_print_time_;

    // Serial
    SerialPort serial_port_;
    bool serial_connected_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace charlie_base_driver