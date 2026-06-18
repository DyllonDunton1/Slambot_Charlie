#include <Arduino.h>
#include <Wire.h>

#include "pins.h"

constexpr uint32_t motor_vel_update_time_us = 10000; // ms
float max_linear_speed = 1.0; // m/s
float max_command_speed = max_linear_speed * 1.1;
float linear_acceleration = 0.001; // m/s

float wheel_radius = 0.05; //meters
float circumference = 2 * 3.141592 * wheel_radius;
float ticks_per_rev = 4096.0f;
float meters_per_tick = circumference / (ticks_per_rev);

constexpr uint8_t RAW_ANGLE_HIGH_REG = 0x0C;
constexpr uint8_t RAW_ANGLE_LOW_REG  = 0x0D;

uint16_t oldRawAngle = 0;
int32_t delta = 0;
bool reading_is_ok = true;
int odom_fail_count = 0;
constexpr int max_odom_fail_count = 5;

float current_speed_target = 0;
float current_speed_final = 0;
float current_speed_odom = 0;
float commanded_speed = 0;

float error = 0.0f;
float integral_error = 0.0f;
float correction = 0.0f;

float kp = 0.2f;
float ki = 0.3f;
float max_error = 0.25;
float max_integral_error = 0.05f;
float max_correction = 0.1;

int counter = 0;
int dir_flag = 0;

int64_t total_ticks = 0;

uint32_t last_us = 0;
float dt_s = 0.01f;

uint16_t read_odom_raw_angle(){
    Wire.beginTransmission(Pins::LeftWheel::ENCODER_I2C_ADDRESS);
    Wire.write(RAW_ANGLE_HIGH_REG);

    uint8_t error = Wire.endTransmission(false); // repeated start

    if (error != 0) {
        Serial.print("I2C write error: ");
        Serial.println(error);
        return 0xFFFF;
    }

    uint8_t bytesRead = Wire.requestFrom(Pins::LeftWheel::ENCODER_I2C_ADDRESS, (uint8_t)2);

    if (bytesRead != 2) {
        Serial.println("I2C read error");
        return 0xFFFF;
    }

    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();

    uint16_t angle = ((uint16_t)highByte << 8) | lowByte;

    angle &= 0x0FFF;

    return angle;
}

void init_encoder() {
    uint16_t angle = read_odom_raw_angle();

    while (angle == 0xFFFF) {
        Serial.println("Waiting for AS5600...");
        delay(100);
        angle = read_odom_raw_angle();
    }

    oldRawAngle = angle;
}

bool update_motor_delta() {
    uint16_t rawAngle = read_odom_raw_angle();

    if (rawAngle == 0xFFFF) {
        // error case, just dont update
        return false;
    }

    delta = (int32_t)rawAngle - (int32_t)oldRawAngle;
    if (delta > 2048) {
        delta -= 4096;
    }
    else if (delta < -2048) {
        delta += 4096;
    }
    oldRawAngle = rawAngle;

    total_ticks += delta;

    return true;
}

float delta_to_speed_mps() {
    return (delta * meters_per_tick) / (dt_s);
}

float delta_to_distance_m() {
    return total_ticks * meters_per_tick;
}

void update_speed_odom() {
    reading_is_ok = update_motor_delta();
    if (reading_is_ok) {
        current_speed_odom = delta_to_speed_mps();
    }
}


void init_motor() {
    //Init Motor Pins
    pinMode(Pins::LeftWheel::STEP_PIN, OUTPUT);
    pinMode(Pins::LeftWheel::DIR_PIN, OUTPUT);
    pinMode(Pins::LeftWheel::ENABLE_PIN, OUTPUT);

    // Set pins for default
    digitalWrite(Pins::LeftWheel::ENABLE_PIN, LOW);
    digitalWrite(Pins::LeftWheel::DIR_PIN, LOW);

    // Set STEP frequency and duty cycle (50% = 128 / 256)
    analogWrite(Pins::LeftWheel::STEP_PIN, 0);
}

void set_motor_freq(float freq) {
    float microsteps = 16.0;
    float micro_freq = freq * microsteps;

    if (micro_freq < 0) {
        digitalWrite(Pins::LeftWheel::DIR_PIN, HIGH);
        micro_freq *= -1.0;
    }
    else {
        digitalWrite(Pins::LeftWheel::DIR_PIN, LOW);
    }

    

    // Set STEP frequency and duty cycle (50% = 128 / 256)
    if (micro_freq > 1.0f) {
        analogWriteFrequency(Pins::LeftWheel::STEP_PIN, micro_freq);
        analogWrite(Pins::LeftWheel::STEP_PIN, 128);
    }
    else {
        analogWrite(Pins::LeftWheel::STEP_PIN, 0);
    }
}

void set_motor_rps(float rps) {
    
    float steps_per_revolution = 200.0;
    float target_frequency = rps * steps_per_revolution;
    set_motor_freq(target_frequency);

}

void set_wheel_speed(float speed) {
    float target_rps = speed / circumference;

    set_motor_rps(target_rps);
}

void update_speed_target() {
    float old_target = current_speed_target;

    if (current_speed_target < current_speed_final) {
        current_speed_target += linear_acceleration;
        if (current_speed_target > current_speed_final) {
            current_speed_target = current_speed_final;
        }
        // bound to max speed
        current_speed_target = constrain(current_speed_target, -max_linear_speed, max_linear_speed);  
    }
    else if (current_speed_target > current_speed_final) {
        current_speed_target -= linear_acceleration;
        if (current_speed_target < current_speed_final) {
            current_speed_target = current_speed_final;
        }
        // bound to max speed
        current_speed_target = constrain(current_speed_target, -max_linear_speed, max_linear_speed);    
    }

    bool crossed_zero =
        (old_target > 0.0f && current_speed_target <= 0.0f) ||
        (old_target < 0.0f && current_speed_target >= 0.0f);

    if (crossed_zero) {
        integral_error = 0.0f;
    }
}

void set_final_speed(float final_speed) {
    // reset on dir change
    if ((current_speed_final > 0.0f && final_speed < 0.0f) ||
        (current_speed_final < 0.0f && final_speed > 0.0f)) {
        integral_error = 0.0f;
    }



    current_speed_final = final_speed;
}

void update_commanded_speed_PI() {
    error = current_speed_target - current_speed_odom;
    error = constrain(error, -max_error, max_error);
    if (abs(current_speed_target) < 0.001f) {
        integral_error = 0.0f;
    }
    else {
        integral_error += error * (dt_s);
    }
    integral_error = constrain(integral_error, -max_integral_error, max_integral_error);
    correction = kp*error + ki*integral_error;

    // Clamp
    correction = constrain(correction, -max_correction, max_correction);

    commanded_speed = current_speed_target + correction;
    commanded_speed = constrain(commanded_speed, -max_command_speed, max_command_speed);
}

void display_speed() {
    Serial.print("Current Speed: ");
    Serial.print(current_speed_odom);
    Serial.println(" m/s");
}

void speed_control_update() {
    update_speed_odom();

    if (reading_is_ok) {
        odom_fail_count = 0;

        //display_speed();
        update_speed_target();
        update_commanded_speed_PI();
        set_wheel_speed(commanded_speed);
    }
    else {
        odom_fail_count++;

        if (odom_fail_count >= max_odom_fail_count) {
            set_wheel_speed(0.0f);
            integral_error = 0.0f;
            correction = 0.0f;
            commanded_speed = 0.0f;

            Serial.println("ODOM FAULT: stopping motor");
        }
    }
}

void setup() {

    Serial.begin(115200);
    delay(1000);

    Wire.begin();
    Wire.setClock(Pins::I2C::CLOCK_HZ);

    init_encoder();
    init_motor();

    last_us = micros();

}

void loop() {

    uint32_t now_us = micros();

    if (now_us - last_us >= motor_vel_update_time_us) {
        uint32_t elapsed_us = now_us - last_us;
        last_us = now_us;

        dt_s = elapsed_us / 1000000.0f;


        if (counter % 500 == 0) {
            if (dir_flag == 0) {
                set_final_speed(0.2);
                dir_flag = 1;
            }
            else if (dir_flag == 1) {
                set_final_speed(0.1);
                dir_flag = 2;
            }
            else if (dir_flag == 2) {
                set_final_speed(0.0);
                dir_flag = 3;
            }
            else if (dir_flag == 3) {
                set_final_speed(-0.1);
                dir_flag = 4;
            }
            else if (dir_flag == 4) {
                set_final_speed(-0.2);
                dir_flag = 5;
            }
            else {
                set_final_speed(0.2);
                dir_flag = 0;
            }
        }

        speed_control_update();
        
        if (counter % 10 == 0) {
            Serial.print("counter: ");
            Serial.print(counter);

            Serial.print(" | total_distance: ");
            Serial.print(delta_to_distance_m(), 3);

            Serial.print(" | final: ");
            Serial.print(current_speed_final, 3);

            Serial.print(" | target: ");
            Serial.print(current_speed_target, 3);

            Serial.print(" | measured: ");
            Serial.print(current_speed_odom, 3);

            Serial.print(" | error: ");
            Serial.print(error, 3);

            Serial.print(" | correction: ");
            Serial.println(correction, 3);

            
        }
        counter++;
    }
}
