#pragma once

#include <Arduino.h>
#include "Motor.h"
#include "Odometer.h"

class Wheel {
public:
    Wheel(Motor& motor, Odometer& odom);

    void init_wheel();
    void update_speed_target();
    void set_final_speed(float final_speed);
    void update_commanded_speed_PI(float dt_s);
    void display_speed();
    void speed_control_update(float dt_s);

    float current_speed_target = 0.0f;
    float current_speed_final = 0.0f;
    float current_speed_odom = 0.0f;
    float commanded_speed = 0.0f;

    float error = 0.0f;
    float integral_error = 0.0f;
    float correction = 0.0f;

    float max_linear_speed = 1.0f;
    float max_command_speed = 1.1f;
    float linear_acceleration = 0.005f;

    float kp = 0.05f;
    float ki = 0.0f;
    float max_error = 0.25f;
    float max_integral_error = 0.05f;
    float max_correction = 0.1f;

    int odom_fail_count = 0;
    static constexpr int max_odom_fail_count = 5;

private:
    Motor& motor;
    Odometer& odom;
};