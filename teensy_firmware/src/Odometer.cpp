#include "Odometer.h"

Odometer::Odometer(TwoWire& wire_bus, uint8_t encoder_i2c_address, float meters_per_tick, bool flip_dir)
    : WireBus(wire_bus),
      ENCODER_I2C_ADDRESS(encoder_i2c_address),
      meters_per_tick(meters_per_tick),
      flip_dir(flip_dir) {}

uint16_t Odometer::read_odom_raw_angle() {
    WireBus.beginTransmission(ENCODER_I2C_ADDRESS);
    WireBus.write(RAW_ANGLE_HIGH_REG);

    uint8_t error = WireBus.endTransmission(false);

    if (error != 0) {
        Serial.print("I2C write error: ");
        Serial.println(error);
        return 0xFFFF;
    }

    uint8_t bytesRead = WireBus.requestFrom(ENCODER_I2C_ADDRESS, (uint8_t)2);

    if (bytesRead != 2) {
        Serial.println("I2C read error");
        return 0xFFFF;
    }

    uint8_t highByte = WireBus.read();
    uint8_t lowByte = WireBus.read();

    uint16_t angle = ((uint16_t)highByte << 8) | lowByte;
    angle &= 0x0FFF;

    return angle;
}

void Odometer::init_encoder() {
    uint16_t angle = read_odom_raw_angle();

    while (angle == 0xFFFF) {
        Serial.println("Waiting for AS5600...");
        delay(100);
        angle = read_odom_raw_angle();
    }

    oldRawAngle = angle;
}

bool Odometer::update_motor_delta() {
    uint16_t rawAngle = read_odom_raw_angle();

    if (rawAngle == 0xFFFF) {
        reading_is_ok = false;
        return false;
    }

    delta = (int32_t)rawAngle - (int32_t)oldRawAngle;

    if (!flip_dir) delta *= -1;

    if (delta > 2048) {
        delta -= 4096;
    }
    else if (delta < -2048) {
        delta += 4096;
    }

    oldRawAngle = rawAngle;
    total_ticks += delta;
    reading_is_ok = true;

    return true;
}

float Odometer::delta_to_speed_mps(float dt_s) {
    if (dt_s <= 0.0f) {
        return 0.0f;
    }

    return (delta * meters_per_tick) / dt_s;
}

float Odometer::delta_to_distance_m() {
    return total_ticks * meters_per_tick;
}

void Odometer::update_speed_odom(float dt_s) {
    reading_is_ok = update_motor_delta();

    if (reading_is_ok) {
        current_speed_odom = delta_to_speed_mps(dt_s);
    }
}
