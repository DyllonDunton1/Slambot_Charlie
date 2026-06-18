#include "Motor.h"

Motor::Motor(uint8_t step_pin,
             uint8_t dir_pin,
             uint8_t enable_pin,
             float circumference,
             bool flip_dir)
    : STEP_PIN(step_pin),
      DIR_PIN(dir_pin),
      ENABLE_PIN(enable_pin),
      circumference(circumference),
      flip_dir(flip_dir) {}

void Motor::init_motor() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    analogWrite(STEP_PIN, 0);
}

void Motor::set_motor_freq(float freq) {
    float micro_freq = freq * microsteps;


    if (micro_freq < 0.0f) {
        digitalWrite(DIR_PIN, flip_dir ? LOW : HIGH);
        micro_freq *= -1.0f;
    }
    else {
        digitalWrite(DIR_PIN, flip_dir ? HIGH : LOW);
    }

    if (micro_freq > 1.0f) {
        analogWriteFrequency(STEP_PIN, micro_freq);
        analogWrite(STEP_PIN, 128);
    }
    else {
        analogWrite(STEP_PIN, 0);
    }
}

void Motor::set_motor_rps(float rps) {
    float target_frequency = rps * steps_per_revolution;
    set_motor_freq(target_frequency);
}

void Motor::set_wheel_speed(float speed) {
    float target_rps = speed / circumference;
    set_motor_rps(target_rps);
}

void Motor::stop_motor() {
    set_wheel_speed(0.0f);
}
