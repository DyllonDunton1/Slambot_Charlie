#pragma once

#include <Arduino.h>

class Motor {
public:
    Motor(uint8_t step_pin,
          uint8_t dir_pin,
          uint8_t enable_pin,
          float circumference,
          bool flip_dir);

    void init_motor();
    void set_motor_freq(float freq);
    void set_motor_rps(float rps);
    void set_wheel_speed(float speed);
    void stop_motor();

private:
    uint8_t STEP_PIN;
    uint8_t DIR_PIN;
    uint8_t ENABLE_PIN;

    float circumference;
    float steps_per_revolution = 200.0f;
    float microsteps = 16.0f;
    bool flip_dir;
};
