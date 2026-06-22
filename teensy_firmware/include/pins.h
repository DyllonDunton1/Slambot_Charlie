#include <Arduino.h>

namespace Pins {

    namespace UsbSerial {
        constexpr uint32_t BAUD_RATE = 115200;
    }

    namespace I2C {
        // Teensy 4.1 default Wire pins: SDA = 18, SCL = 19
        constexpr uint8_t SDA_PIN = 18;
        constexpr uint8_t SCL_PIN = 19;
        constexpr uint32_t CLOCK_HZ = 400000;
    }

    namespace LeftWheel {
        constexpr uint8_t STEP_PIN   = 29;
        constexpr uint8_t DIR_PIN    = 31;
        constexpr uint8_t ENABLE_PIN = 32;

        constexpr uint8_t ENCODER_I2C_ADDRESS = 0x36;
    }

    namespace RightWheel {
        constexpr uint8_t STEP_PIN   = 1;
        constexpr uint8_t DIR_PIN    = 2;
        constexpr uint8_t ENABLE_PIN = 3;

        constexpr uint8_t ENCODER_I2C_ADDRESS = 0x36;
    }

    namespace StatusLED {
        constexpr uint8_t ONBOARD_LED_PIN = 13;
    }

}
