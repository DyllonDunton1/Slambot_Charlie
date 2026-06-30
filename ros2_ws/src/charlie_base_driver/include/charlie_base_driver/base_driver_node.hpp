#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/battery_state.hpp"

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
    void handle_debug_packet(const std::string & line);
    void handle_battery_packet(const std::string & line);
    double estimate_battery_percentage(double voltage) const;
    void publish_battery_state(double voltage);

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

    void tuning_command_callback(const std_msgs::msg::String::SharedPtr msg);
    void send_teensy_config_command(const std::string & command);
    void update_tuning_debug_json(
        std::ostringstream & json,
        double kp,
        double ki,
        double wheel_radius_m);
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters);

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

    // Batery monitoring
    double last_battery_voltage_;
    double last_battery_percentage_;

    // Timing
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_debug_print_time_;

    // Serial
    SerialPort serial_port_;
    bool serial_connected_;

    // Params
    double last_kp_;
    double last_ki_;
    double last_wheel_radius_m_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr base_debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tuning_command_sub_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace charlie_base_driver