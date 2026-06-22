"""
Vibration Feature Node.

Subscribes to /imu/raw, maintains a sliding window of accelerometer
magnitude samples, and publishes the standard deviation of that
window on /vibration_feature as a std_msgs/Float32 message.  The
feature is the single scalar input to the fuzzy controller in
fuzzy_alpha_node.py.

This node corresponds to:
  - Chapter 3, Section 3.6 (vibration feature design)
  - Chapter 4, Section 4.3 (implementation details)

The feature is defined formally in equation (3.4) of Section 3.6.2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

import numpy as np
from collections import deque


class VibrationFeatureNode(Node):
    def __init__(self):
        super().__init__('vibration_feature_node')

        # The default window_size of 50 is the value used in the
        # second tuning iteration described in Section 4.3.3.  The
        # final value W = 70 used for all experimental results is
        # set by the launch file at runtime; the rationale and the
        # supporting window-length sensitivity analysis are in
        # Section 3.6.3 (Figure 3.6 and Table 3.2).
        self.declare_parameter('input_topic', '/imu/raw')
        self.declare_parameter('output_topic', '/vibration_feature')
        self.declare_parameter('window_size', 50)
        self.declare_parameter('convert_to_g', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        self.convert_to_g = self.get_parameter('convert_to_g').get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Float32, output_topic, 10)

        # Sliding buffer of accelerometer magnitudes.  collections.deque
        # with maxlen=W gives O(1) append with automatic eviction at the
        # opposite end when full; this is the implementation pattern
        # discussed in Section 4.3.1.
        self.acc_buffer = deque(maxlen=self.window_size)
        self.g_to_ms2 = 9.81

        self.get_logger().info(
            f'Vibration feature node started | input_topic={input_topic}, '
            f'output_topic={output_topic}, window_size={self.window_size}, convert_to_g={self.convert_to_g}'
        )

    def listener_callback(self, msg):
        ax_ms2 = msg.linear_acceleration.x
        ay_ms2 = msg.linear_acceleration.y
        az_ms2 = msg.linear_acceleration.z

        # Convert m/s^2 -> g.  The acquisition node (imu_reader_node)
        # multiplied raw-count/16384 by 9.81 to produce SI units; here
        # we reverse the same factor so the magnitude accumulated in
        # the sliding window is in g.  The two conversions cancel
        # exactly, see Section 4.3.2.
        if self.convert_to_g:
            ax = ax_ms2 / self.g_to_ms2
            ay = ay_ms2 / self.g_to_ms2
            az = az_ms2 / self.g_to_ms2
        

        # Acceleration magnitude.  Orientation-invariant by construction
        # (Section 3.6.1).  This is the |a_k| of equation (3.1)
        # in Section 3.6.2.
        a_mag = np.sqrt(ax**2 + ay**2 + az**2)

        self.acc_buffer.append(a_mag)

        # Do not publish until the sliding window is full.  At W = 70
        # and 100 Hz, this introduces a ~0.7 s startup delay
        # (Section 4.3.1), accommodated in the experimental protocol
        # by the warm-up interval described in Section 5.1.
        if len(self.acc_buffer) == self.window_size:

            # Standard deviation of the sliding-window magnitudes.
            # This is the v_k of equation (3.3) (Section 3.6.2).
            # numpy.std uses the population definition (divisor W, not
            # W-1), which matches the formula in the thesis.  The mean
            # is recomputed at every sample by numpy.std, making the
            # feature insensitive to slow drifts in the underlying
            # signal.
            std_val = float(np.std(self.acc_buffer))

            out_msg = Float32()
            out_msg.data = std_val

            self.publisher.publish(out_msg)

            self.get_logger().info(f'Feature std(|a|): {std_val:.4f}')


def main(args=None):
    rclpy.init(args=args)
    node = VibrationFeatureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()