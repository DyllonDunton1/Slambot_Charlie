#include <Arduino.h>
#include <Wire.h>

#include "pins.h"
#include "Motor.h"
#include "Odometer.h"
#include "Wheel.h"

constexpr uint32_t motor_vel_update_time_us = 10000;

constexpr float wheel_radius = 0.0425f;
constexpr float circumference = 2.0f * 3.14159265f * wheel_radius;
constexpr float ticks_per_rev = 4096.0f;
constexpr float meters_per_tick = circumference / ticks_per_rev;

constexpr bool left_flip = false;
constexpr bool right_flip = true;

constexpr uint32_t serial_status_update_time_us = 20000;  // 50 Hz
constexpr uint32_t serial_command_timeout_us = 500000;    // 0.5 s

uint32_t last_serial_status_us = 0;
uint32_t last_serial_command_us = 0;

constexpr uint32_t serial_debug_update_time_us = 100000;  // 10 Hz
uint32_t last_serial_debug_us = 0;


String serial_line = "";

float left_command_mps = 0.0f;
float right_command_mps = 0.0f;

bool serial_connected_once = false;
uint32_t status_flags = 0;

constexpr uint32_t STATUS_OK = 0;
constexpr uint32_t STATUS_SERIAL_TIMEOUT = 1 << 0;
constexpr uint32_t STATUS_LEFT_ODOM_ERROR = 1 << 1;
constexpr uint32_t STATUS_RIGHT_ODOM_ERROR = 1 << 2;

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




void handle_serial_line(const String &line) {
    if (line.length() == 0) {
        return;
    }

    char command = line.charAt(0);

    if (command == 'V') {
        float left = 0.0f;
        float right = 0.0f;

        int parsed = sscanf(line.c_str(), "V %f %f", &left, &right);

        if (parsed == 2) {
            left_command_mps = left;
            right_command_mps = right;

            left_wheel.set_final_speed(left_command_mps);
            right_wheel.set_final_speed(right_command_mps);

            last_serial_command_us = micros();
            serial_connected_once = true;

            status_flags &= ~STATUS_SERIAL_TIMEOUT;
        }
        else {
            Serial.println("E BAD_V_COMMAND");
        }
    }
    else if (command == 'S') {
        left_command_mps = 0.0f;
        right_command_mps = 0.0f;

        left_wheel.set_final_speed(0.0f);
        right_wheel.set_final_speed(0.0f);

        last_serial_command_us = micros();

        Serial.println("A STOP");
    }
    else {
        Serial.println("E UNKNOWN_COMMAND");
    }
}

void read_serial_commands() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n') {
            serial_line.trim();

            if (serial_line.length() > 0) {
                handle_serial_line(serial_line);
            }

            serial_line = "";
        }
        else if (c != '\r') {
            serial_line += c;

            // Prevent runaway buffer if something bad happens.
            if (serial_line.length() > 80) {
                serial_line = "";
            }
        }
    }
}

void check_serial_timeout() {
    uint32_t now_us = micros();

    if (serial_connected_once && (now_us - last_serial_command_us > serial_command_timeout_us)) {
        left_command_mps = 0.0f;
        right_command_mps = 0.0f;

        left_wheel.set_final_speed(0.0f);
        right_wheel.set_final_speed(0.0f);

        status_flags |= STATUS_SERIAL_TIMEOUT;
    }
}

void send_odom_status() {
    uint32_t now_us = micros();

    if (now_us - last_serial_status_us < serial_status_update_time_us) {
        return;
    }

    last_serial_status_us = now_us;

    Serial.print("O ");

    Serial.print(left_odom.delta_to_distance_m(),4);
    Serial.print(" ");

    Serial.print(right_odom.delta_to_distance_m(),4);
    Serial.print(" ");

    Serial.print(left_wheel.current_speed_odom, 3);
    Serial.print(" ");

    Serial.print(right_wheel.current_speed_odom, 3);
    Serial.print(" ");

    Serial.println(status_flags);
}

void send_debug_status() {

    uint32_t debug_now_us = micros();

    if (debug_now_us - last_serial_debug_us < serial_debug_update_time_us) {
        return;
    }

    last_serial_debug_us = debug_now_us;

    Serial.print("D ");

    Serial.print(left_wheel.current_speed_target, 3);
    Serial.print(" ");

    Serial.print(right_wheel.current_speed_target, 3);
    Serial.print(" ");

    Serial.print(left_wheel.current_speed_odom, 3);
    Serial.print(" ");

    Serial.print(right_wheel.current_speed_odom, 3);
    Serial.print(" ");

    Serial.print(left_wheel.error, 3);
    Serial.print(" ");

    Serial.print(right_wheel.error, 3);
    Serial.print(" ");

    Serial.print(left_wheel.integral_error, 3);
    Serial.print(" ");

    Serial.print(right_wheel.integral_error, 3);
    Serial.print(" ");

    Serial.print(left_wheel.commanded_speed, 3);
    Serial.print(" ");

    Serial.println(right_wheel.commanded_speed, 3);


}

void setup() {
    Serial.begin(115200);

    // Do not wait forever for Serial on a robot.
    // This gives the Pi time to connect, but still boots standalone.
    uint32_t serial_start_ms = millis();
    while (!Serial && (millis() - serial_start_ms < 2000)) {
        delay(10);
    }

    last_serial_command_us = micros();
    last_serial_status_us = micros();

    Wire.begin();
    Wire.setClock(Pins::I2C::CLOCK_HZ);

    Wire1.begin();
    Wire1.setClock(Pins::I2C::CLOCK_HZ);

    left_wheel.init_wheel();
    right_wheel.init_wheel();

    last_us = micros();

    digitalWrite(Pins::StatusLED::ONBOARD_LED_PIN, HIGH);
}

void loop() {
    uint32_t now_us = micros();

    read_serial_commands();
    check_serial_timeout();

    if (now_us - last_us >= motor_vel_update_time_us) {
        uint32_t elapsed_us = now_us - last_us;
        last_us = now_us;

        dt_s = elapsed_us / 1000000.0f;

        left_wheel.speed_control_update(dt_s);
        right_wheel.speed_control_update(dt_s);
    }

    send_odom_status();
    send_debug_status();
}