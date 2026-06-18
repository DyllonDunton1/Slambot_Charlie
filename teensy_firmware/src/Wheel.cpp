#include "Wheel.h"

Wheel::Wheel(Motor& motor, Odometer& odom)
    : motor(motor),
      odom(odom) {}

void Wheel::init_wheel() {
    odom.init_encoder();
    motor.init_motor();
}

void Wheel::update_speed_target() {
    float old_target = current_speed_target;

    if (current_speed_target < current_speed_final) {
        current_speed_target += linear_acceleration;

        if (current_speed_target > current_speed_final) {
            current_speed_target = current_speed_final;
        }

        current_speed_target = constrain(current_speed_target, -max_linear_speed, max_linear_speed);
    }
    else if (current_speed_target > current_speed_final) {
        current_speed_target -= linear_acceleration;

        if (current_speed_target < current_speed_final) {
            current_speed_target = current_speed_final;
        }

        current_speed_target = constrain(current_speed_target, -max_linear_speed, max_linear_speed);
    }

    bool crossed_zero =
        (old_target > 0.0f && current_speed_target <= 0.0f) ||
        (old_target < 0.0f && current_speed_target >= 0.0f);

    if (crossed_zero) {
        integral_error = 0.0f;
    }
}

void Wheel::set_final_speed(float final_speed) {
    if ((current_speed_final > 0.0f && final_speed < 0.0f) ||
        (current_speed_final < 0.0f && final_speed > 0.0f)) {
        integral_error = 0.0f;
    }

    current_speed_final = final_speed;
}

void Wheel::update_commanded_speed_PI(float dt_s) {
    error = current_speed_target - current_speed_odom;
    error = constrain(error, -max_error, max_error);

    if (abs(current_speed_target) < 0.001f) {
        integral_error = 0.0f;
    }
    else {
        integral_error += error * dt_s;
    }

    integral_error = constrain(integral_error, -max_integral_error, max_integral_error);

    correction = kp * error + ki * integral_error;
    correction = constrain(correction, -max_correction, max_correction);

    commanded_speed = current_speed_target + correction;
    commanded_speed = constrain(commanded_speed, -max_command_speed, max_command_speed);
}

void Wheel::display_speed() {
    Serial.print("Current Speed: ");
    Serial.print(current_speed_odom);
    Serial.println(" m/s");
}

void Wheel::speed_control_update(float dt_s) {
    odom.update_speed_odom(dt_s);
    current_speed_odom = odom.current_speed_odom;

    if (odom.reading_is_ok) {
        odom_fail_count = 0;

        update_speed_target();
        update_commanded_speed_PI(dt_s);
        motor.set_wheel_speed(commanded_speed);
    }
    else {
        odom_fail_count++;

        if (odom_fail_count >= max_odom_fail_count) {
            motor.stop_motor();
            integral_error = 0.0f;
            correction = 0.0f;
            commanded_speed = 0.0f;

            Serial.println("ODOM FAULT: stopping motor");
        }
    }
}
