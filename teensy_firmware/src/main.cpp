#include <Arduino.h>
#include <Wire.h>

#include "pins.h"
#include "Motor.h"
#include "Odometer.h"
#include "Wheel.h"

constexpr uint32_t motor_vel_update_time_us = 10000;

constexpr float wheel_radius = 0.041f;
constexpr float circumference = 2.0f * 3.14159265f * wheel_radius;
constexpr float ticks_per_rev = 4096.0f;
constexpr float meters_per_tick = circumference / ticks_per_rev;

constexpr bool left_flip = true;
constexpr bool right_flip = false;

// Left AS5600 on default I2C bus: SDA/SCL
Odometer left_odom(
    Wire,
    Pins::LeftWheel::ENCODER_I2C_ADDRESS,
    meters_per_tick,
    left_flip
);

// Right AS5600 on second I2C bus: SDA1/SCL1
Odometer right_odom(
    Wire1,
    Pins::RightWheel::ENCODER_I2C_ADDRESS,
    meters_per_tick,
    right_flip
);

Motor left_motor(
    Pins::LeftWheel::STEP_PIN,
    Pins::LeftWheel::DIR_PIN,
    Pins::LeftWheel::ENABLE_PIN,
    circumference,
    left_flip
);

Motor right_motor(
    Pins::RightWheel::STEP_PIN,
    Pins::RightWheel::DIR_PIN,
    Pins::RightWheel::ENABLE_PIN,
    circumference,
    right_flip
);

Wheel left_wheel(left_motor, left_odom);
Wheel right_wheel(right_motor, right_odom);


void set_speeds(float left_speed, float right_speed) {
    left_wheel.set_final_speed(left_speed);
    right_wheel.set_final_speed(right_speed);
}


uint32_t last_us = 0;
float dt_s = 0.01f;

int counter = 0;
int dir_flag = 0;

void update_test_speed() {
    if (counter % 275 == 0) {
        if (dir_flag == 0) {
            set_speeds(0.2f, 0.2f);
            dir_flag = 1;
        }
        else if (dir_flag == 1) {
            set_speeds(0.0f, 0.0f);
            dir_flag = 2;
        }
        else if (dir_flag == 2) {
            set_speeds(0.2f, -0.2f);
            dir_flag = 3;
        }
        else {
            set_speeds(0.0f, 0.0f);
            dir_flag = 0;
        }
    }
}

void print_debug() {
    Serial.print("counter: ");
    Serial.print(counter);

    Serial.print(" | L_dist: ");
    Serial.print(left_odom.delta_to_distance_m(), 3);

    Serial.print(" | L_final: ");
    Serial.print(left_wheel.current_speed_final, 3);

    Serial.print(" | L_target: ");
    Serial.print(left_wheel.current_speed_target, 3);

    Serial.print(" | L_measured: ");
    Serial.print(left_wheel.current_speed_odom, 3);

    Serial.print(" | L_error: ");
    Serial.print(left_wheel.error, 3);

    Serial.print(" | L_corr: ");
    Serial.print(left_wheel.correction, 3);

    Serial.print(" || R_dist: ");
    Serial.print(right_odom.delta_to_distance_m(), 3);

    Serial.print(" | R_final: ");
    Serial.print(right_wheel.current_speed_final, 3);

    Serial.print(" | R_target: ");
    Serial.print(right_wheel.current_speed_target, 3);

    Serial.print(" | R_measured: ");
    Serial.print(right_wheel.current_speed_odom, 3);

    Serial.print(" | R_error: ");
    Serial.print(right_wheel.error, 3);

    Serial.print(" | R_corr: ");
    Serial.println(right_wheel.correction, 3);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin();
    Wire.setClock(Pins::I2C::CLOCK_HZ);

    Wire1.begin();
    Wire1.setClock(Pins::I2C::CLOCK_HZ);

    left_wheel.init_wheel();
    right_wheel.init_wheel();

    last_us = micros();
}

void loop() {
    uint32_t now_us = micros();

    if (now_us - last_us >= motor_vel_update_time_us) {
        uint32_t elapsed_us = now_us - last_us;
        last_us = now_us;

        dt_s = elapsed_us / 1000000.0f;

        update_test_speed();

        left_wheel.speed_control_update(dt_s);
        right_wheel.speed_control_update(dt_s);

        if (counter % 10 == 0) {
            print_debug();
        }

        counter++;
    }
}