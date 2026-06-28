#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import smbus
except ImportError as exc:
    raise ImportError(
        "Missing smbus. Install with: sudo apt install python3-smbus"
    ) from exc


class CharlieImuNode(Node):
    # ICM-20948 register banks
    REG_BANK_SEL = 0x7F

    # Bank 0
    WHO_AM_I = 0x00
    PWR_MGMT_1 = 0x06
    PWR_MGMT_2 = 0x07
    GYRO_ZOUT_H = 0x37

    # Bank 2
    GYRO_SMPLRT_DIV = 0x00
    GYRO_CONFIG_1 = 0x01

    EXPECTED_WHO_AM_I = 0xEA

    DEG_TO_RAD = math.pi / 180.0

    # This matches gyro full-scale range +/-250 deg/s.
    GYRO_LSB_PER_DPS = 131.0

    def __init__(self):
        super().__init__("charlie_imu_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_addr", 0x69)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("gyro_z_sign", 1.0)
        self.declare_parameter("bias_samples", 250)

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.i2c_addr = int(self.get_parameter("i2c_addr").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.gyro_z_sign = float(self.get_parameter("gyro_z_sign").value)
        self.bias_samples = int(self.get_parameter("bias_samples").value)

        self.bus = smbus.SMBus(self.i2c_bus)
        self.current_bank = None

        self.init_imu()

        self.get_logger().info("Keep robot still: calibrating gyro Z bias...")
        self.gyro_z_bias = self.calibrate_gyro_z_bias(self.bias_samples)
        self.get_logger().info(f"Gyro Z bias: {self.gyro_z_bias:.6f} rad/s")

        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.publish_imu)

    def select_bank(self, bank: int):
        if self.current_bank == bank:
            return

        self.bus.write_byte_data(self.i2c_addr, self.REG_BANK_SEL, bank << 4)
        self.current_bank = bank

    def write_reg(self, reg: int, value: int):
        self.bus.write_byte_data(self.i2c_addr, reg, value & 0xFF)

    def read_reg(self, reg: int) -> int:
        return self.bus.read_byte_data(self.i2c_addr, reg)

    @staticmethod
    def to_int16(msb: int, lsb: int) -> int:
        value = (msb << 8) | lsb
        if value & 0x8000:
            value -= 0x10000
        return value

    def init_imu(self):
        self.select_bank(0)

        who_am_i = self.read_reg(self.WHO_AM_I)
        if who_am_i != self.EXPECTED_WHO_AM_I:
            raise RuntimeError(
                f"ICM-20948 not found at 0x{self.i2c_addr:02X}. "
                f"WHO_AM_I got 0x{who_am_i:02X}, expected 0x{self.EXPECTED_WHO_AM_I:02X}"
            )

        self.get_logger().info(
            f"Connected to ICM-20948 at 0x{self.i2c_addr:02X}"
        )

        # Reset IMU.
        self.write_reg(self.PWR_MGMT_1, 0x80)
        time.sleep(0.10)

        # Wake IMU and select clock.
        self.write_reg(self.PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        # Enable accel and gyro axes.
        # Even though we only use gyro Z, this leaves the device awake normally.
        self.write_reg(self.PWR_MGMT_2, 0x00)
        time.sleep(0.05)

        self.select_bank(2)

        # Gyro sample-rate divider.
        # Good enough for a 100 Hz ROS publisher.
        self.write_reg(self.GYRO_SMPLRT_DIV, 10)

        # Gyro DLPF config = 3, full-scale = +/-250 deg/s, DLPF enabled.
        self.write_reg(self.GYRO_CONFIG_1, (3 << 3) | (0 << 1) | 1)

        self.select_bank(0)

    def read_gyro_z(self) -> float:
        self.select_bank(0)

        data = self.bus.read_i2c_block_data(self.i2c_addr, self.GYRO_ZOUT_H, 2)
        gz_raw = self.to_int16(data[0], data[1])

        gz_dps = gz_raw / self.GYRO_LSB_PER_DPS
        gz_rad_s = gz_dps * self.DEG_TO_RAD

        return gz_rad_s

    def calibrate_gyro_z_bias(self, samples: int) -> float:
        total = 0.0

        for _ in range(samples):
            total += self.read_gyro_z()
            time.sleep(0.01)

        return total / float(samples)

    def publish_imu(self):
        gz = self.read_gyro_z()

        # Bias correction and sign convention correction.
        gz -= self.gyro_z_bias
        gz *= self.gyro_z_sign

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # We are not publishing orientation.
        msg.orientation_covariance[0] = -1.0

        # We are not using linear acceleration yet.
        msg.linear_acceleration_covariance[0] = -1.0

        # We only care about yaw rate for Charlie right now.
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = gz

        # Large covariance for unused roll/pitch rates.
        # Reasonable starter covariance for yaw rate.
        msg.angular_velocity_covariance[0] = 9999.0
        msg.angular_velocity_covariance[4] = 9999.0
        msg.angular_velocity_covariance[8] = 0.01

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CharlieImuNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()