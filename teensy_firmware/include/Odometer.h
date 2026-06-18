#pragma once

#include <Arduino.h>
#include <Wire.h>

class Odometer {
public:
    Odometer(TwoWire& wire_bus, uint8_t encoder_i2c_address, float meters_per_tick, bool flip_dir);

    void init_encoder();
    uint16_t read_odom_raw_angle();
    bool update_motor_delta();
    float delta_to_speed_mps(float dt_s);
    float delta_to_distance_m();
    void update_speed_odom(float dt_s);

    bool reading_is_ok = true;
    int32_t delta = 0;
    int64_t total_ticks = 0;
    float current_speed_odom = 0.0f;

private:
    TwoWire& WireBus;
    uint8_t ENCODER_I2C_ADDRESS;
    float meters_per_tick;
    bool flip_dir;

    uint16_t oldRawAngle = 0;

    static constexpr uint8_t RAW_ANGLE_HIGH_REG = 0x0C;
    static constexpr uint8_t RAW_ANGLE_LOW_REG  = 0x0D;
};
