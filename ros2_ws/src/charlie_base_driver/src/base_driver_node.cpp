#include "charlie_base_driver/base_driver_node.hpp"

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace charlie_base_driver {

BaseDriverNode::BaseDriverNode()
: Node("base_driver_node"),
  serial_port_name_(""),
  baud_rate_(0),
  wheel_separation_(0.0),
  cmd_timeout_s_(0.0),
  command_rate_hz_(0.0),
  odom_frame_("odom"),
  base_frame_("base_link"),
  publish_tf_(true),
  cmd_linear_x_(0.0),
  cmd_angular_z_(0.0),
  left_target_mps_(0.0),
  right_target_mps_(0.0),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  received_first_odom_(false),
  previous_left_total_m_(0.0),
  previous_right_total_m_(0.0),
  last_status_(0),
  serial_connected_(false)
{
    declare_parameters();
    load_parameters();

    last_cmd_time_ = this->now();
    last_debug_print_time_ = this->now();
    last_odom_time_ = this->now();

    open_serial();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&BaseDriverNode::cmd_vel_callback, this, std::placeholders::_1)
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    const auto timer_period_ms =
        static_cast<int>(1000.0 / command_rate_hz_);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&BaseDriverNode::control_update, this)
    );

    RCLCPP_INFO(this->get_logger(), "Charlie base driver started");
    RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "baud_rate: %d", baud_rate_);
    RCLCPP_INFO(this->get_logger(), "wheel_separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "cmd_timeout_s: %.3f s", cmd_timeout_s_);
    RCLCPP_INFO(this->get_logger(), "command_rate_hz: %.1f Hz", command_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publish_tf_ ? "true" : "false");
}

void BaseDriverNode::declare_parameters()
{
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("wheel_separation", 0.220);
    this->declare_parameter<double>("cmd_timeout_s", 0.500);
    this->declare_parameter<double>("command_rate_hz", 50.0);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("publish_tf", true);
}

void BaseDriverNode::load_parameters()
{
    serial_port_name_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    cmd_timeout_s_ = this->get_parameter("cmd_timeout_s").as_double();
    command_rate_hz_ = this->get_parameter("command_rate_hz").as_double();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    if (command_rate_hz_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "command_rate_hz must be positive. Using 50 Hz.");
        command_rate_hz_ = 50.0;
    }

    if (wheel_separation_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "wheel_separation must be positive. Using 0.220 m.");
        wheel_separation_ = 0.220;
    }

    if (cmd_timeout_s_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "cmd_timeout_s must be positive. Using 0.500 s.");
        cmd_timeout_s_ = 0.500;
    }
}

void BaseDriverNode::open_serial()
{
    serial_connected_ = serial_port_.open_port(serial_port_name_, baud_rate_);

    if (serial_connected_) {
        RCLCPP_INFO(this->get_logger(), "Opened serial port %s", serial_port_name_.c_str());
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port_name_.c_str());
    }
}

void BaseDriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_linear_x_ = msg->linear.x;
    cmd_angular_z_ = msg->angular.z;
    last_cmd_time_ = this->now();

    update_target_wheel_speeds();
}

void BaseDriverNode::control_update()
{
    apply_cmd_vel_timeout();
    update_target_wheel_speeds();
    send_speed_command();
    read_serial();
}

void BaseDriverNode::update_target_wheel_speeds()
{
    left_target_mps_ =
        cmd_linear_x_ - (cmd_angular_z_ * wheel_separation_ / 2.0);

    right_target_mps_ =
        cmd_linear_x_ + (cmd_angular_z_ * wheel_separation_ / 2.0);
}

void BaseDriverNode::apply_cmd_vel_timeout()
{
    const double time_since_cmd_s = (this->now() - last_cmd_time_).seconds();

    if (time_since_cmd_s > cmd_timeout_s_) {
        cmd_linear_x_ = 0.0;
        cmd_angular_z_ = 0.0;
    }
}

void BaseDriverNode::send_speed_command()
{
    const std::string command = make_speed_command_string();

    if (serial_connected_) {
        const bool success = serial_port_.write_string(command);

        if (!success) {
            RCLCPP_WARN(this->get_logger(), "Serial write failed");
            serial_connected_ = false;
            serial_port_.close_port();
        }
    }

    const double time_since_debug_s =
        (this->now() - last_debug_print_time_).seconds();

    if (time_since_debug_s >= 0.5) {
        RCLCPP_INFO(this->get_logger(), "Teensy command: %s", command.c_str());
        last_debug_print_time_ = this->now();

        if (!serial_connected_) {
            RCLCPP_WARN(this->get_logger(), "Serial is not connected");
        }
    }
}

void BaseDriverNode::read_serial()
{
    if (!serial_connected_) {
        return;
    }

    std::string line;

    while (serial_port_.read_line(line)) {
        handle_teensy_line(line);
    }
}

void BaseDriverNode::handle_teensy_line(const std::string & line)
{
    if (line.empty()) {
        return;
    }

    if (line[0] == 'O') {
        char packet_type = '\0';
        double left_total_m = 0.0;
        double right_total_m = 0.0;
        double left_speed_mps = 0.0;
        double right_speed_mps = 0.0;
        int status = 0;

        std::istringstream ss(line);

        ss >> packet_type
           >> left_total_m
           >> right_total_m
           >> left_speed_mps
           >> right_speed_mps
           >> status;

        if (!ss.fail() && packet_type == 'O') {
            process_odom_packet(
                left_total_m,
                right_total_m,
                left_speed_mps,
                right_speed_mps,
                status
            );
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Bad odom packet: %s", line.c_str());
        }
    }
    else if (line[0] == 'E') {
        RCLCPP_WARN(this->get_logger(), "Teensy error: %s", line.c_str());
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Teensy says: %s", line.c_str());
    }
}

void BaseDriverNode::process_odom_packet(
    double left_total_m,
    double right_total_m,
    double left_speed_mps,
    double right_speed_mps,
    int status)
{
    const rclcpp::Time now = this->now();
    last_status_ = status;

    if (!received_first_odom_) {
        previous_left_total_m_ = left_total_m;
        previous_right_total_m_ = right_total_m;
        last_odom_time_ = now;
        received_first_odom_ = true;

        RCLCPP_INFO(this->get_logger(), "Received first odom packet");
        return;
    }

    const double delta_left_m = left_total_m - previous_left_total_m_;
    const double delta_right_m = right_total_m - previous_right_total_m_;

    previous_left_total_m_ = left_total_m;
    previous_right_total_m_ = right_total_m;

    const double delta_center_m = (delta_left_m + delta_right_m) / 2.0;
    const double delta_theta_rad = (delta_right_m - delta_left_m) / wheel_separation_;

    x_ += delta_center_m * std::cos(theta_ + delta_theta_rad / 2.0);
    y_ += delta_center_m * std::sin(theta_ + delta_theta_rad / 2.0);
    theta_ += delta_theta_rad;
    normalize_theta();

    const double linear_velocity_mps =
        (left_speed_mps + right_speed_mps) / 2.0;

    const double angular_velocity_radps =
        (right_speed_mps - left_speed_mps) / wheel_separation_;

    publish_odom(now, linear_velocity_mps, angular_velocity_radps);
}

void BaseDriverNode::publish_odom(
    const rclcpp::Time & stamp,
    double linear_velocity_mps,
    double angular_velocity_radps)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    q.normalize();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = linear_velocity_mps;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity_radps;

    // Simple starter covariance values.
    // These can be tuned later for robot_localization.
    for (int i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }

    odom_msg.pose.covariance[0] = 0.02;    // x
    odom_msg.pose.covariance[7] = 0.02;    // y
    odom_msg.pose.covariance[35] = 0.10;   // yaw

    odom_msg.twist.covariance[0] = 0.05;   // vx
    odom_msg.twist.covariance[35] = 0.20;  // wz

    odom_pub_->publish(odom_msg);

    if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = stamp;
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }
}

void BaseDriverNode::normalize_theta()
{
    while (theta_ > M_PI) {
        theta_ -= 2.0 * M_PI;
    }

    while (theta_ < -M_PI) {
        theta_ += 2.0 * M_PI;
    }
}

std::string BaseDriverNode::make_speed_command_string() const
{
    std::ostringstream ss;

    ss << std::fixed << std::setprecision(3);
    ss << "V "
       << left_target_mps_
       << " "
       << right_target_mps_
       << "\n";

    return ss.str();
}

}  // namespace charlie_base_driver

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<charlie_base_driver::BaseDriverNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}