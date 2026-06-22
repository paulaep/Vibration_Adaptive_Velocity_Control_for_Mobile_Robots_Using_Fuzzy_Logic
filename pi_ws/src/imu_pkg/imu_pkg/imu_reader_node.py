"""
IMU Reader Node.

Reads the MPU-9250 inertial measurement unit over the I2C bus at a
target rate of 100 Hz and publishes the result on the /imu/raw topic
as a sensor_msgs/Imu message.

This node corresponds to:
  - Chapter 3, Section 3.2.3 (MPU-9250 sensor description)
  - Chapter 3, Section 3.4.1 (node decomposition)
  - Chapter 4, Section 4.2 (implementation details)

Two implementation disclosures from Chapter 4 are realized in this file:
the ACCEL_CONFIG register is not written (Section 4.2.2), so the sensor
remains at its power-on default range of +/- 2g but carries an
uncalibrated per-axis bias; and the ACCEL_CONFIG2 register is also not
written (Section 4.2.3), leaving the digital low-pass filter at a
cutoff above the Nyquist limit imposed by the 100 Hz polling rate.
"""

import smbus

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuReaderNode(Node):
    def __init__(self):
        super().__init__('imu_reader_node')

        # Publisher and 100 Hz timer.  The 10 ms inter-sample interval
        # is the budget discussed in Section 4.2.1.
        self.publisher_ = self.create_publisher(Imu, '/imu/raw', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # target ~100 Hz

        # I2C interface on bus 1, MPU-9250 default address 0x68 (Section 3.2.3).
        self.bus = smbus.SMBus(1)
        self.address = 0x68

        # Sensitivity factor for the +/- 2g full-scale range
        # (16384 counts per g).  The ACCEL_CONFIG register at 0x1C is
        # NOT written below, so the sensor remains at its default range,
        # which is also +/- 2g.  The uncalibrated bias disclosure is in
        # Section 4.2.2.
        self.accel_scale = 16384.0
        self.g_to_ms2 = 9.81

        # Wake MPU-9250 from default sleep state by clearing the
        # PWR_MGMT_1 register (0x6B).  Note: ACCEL_CONFIG (0x1C) and
        # ACCEL_CONFIG2 (0x1D) are intentionally not written here;
        # see Sections 4.2.2 and 4.2.3 for the disclosure of the
        # consequences.
        self.bus.write_byte_data(self.address, 0x6B, 0)

        self.get_logger().info('IMU reader node started.')

    def read_word(self, reg):
        """Assemble a signed 16-bit value from two consecutive 8-bit
        registers (high byte first), with manual two's-complement sign
        extension.  smbus returns unsigned bytes only."""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low

        if value >= 0x8000:
            value -= 65536

        return value

    def timer_callback(self):
        try:
            # Read accelerometer X, Y, Z output registers
            # (0x3B/3D/3F).  Gyroscope (0x43+) and magnetometer
            # registers are not read in this implementation.
            ax_raw = self.read_word(0x3B)
            ay_raw = self.read_word(0x3D)
            az_raw = self.read_word(0x3F)

            # Convert raw integer counts to g.  The vibration_feature_node
            # later reverses the g->m/s^2 conversion below so that the
            # values arriving at the standard-deviation computation are
            # back in g; see Section 4.3.2.
            ax_g = ax_raw / self.accel_scale
            ay_g = ay_raw / self.accel_scale
            az_g = az_raw / self.accel_scale

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # Publish in m/s^2 per the sensor_msgs/Imu SI convention.
            msg.linear_acceleration.x = ax_g * self.g_to_ms2
            msg.linear_acceleration.y = ay_g * self.g_to_ms2
            msg.linear_acceleration.z = az_g * self.g_to_ms2

            # Gyroscope and orientation fields are not used by the
            # controller pipeline; zeroed for protocol compliance only.
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0

            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'IMU read failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()