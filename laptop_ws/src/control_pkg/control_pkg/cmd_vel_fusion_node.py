"""
Command Fusion Node.

Subscribes to the operator's velocity command on /cmd_vel_user and the
fuzzy controller's scaling factor on /alpha, applies the shared-control
modulation, and publishes the resulting velocity command on
/S2R5/cmd_vel as a geometry_msgs/Twist message.

This is the node at which the conceptual shared-control modulation
defined in Chapter 3 is realized in operation.  The architectural
behaviour (modulation formula, fixed-rate publication, command timeout,
shutdown safeguard) is presented in Chapter 3; this implementation
realizes it.

This node corresponds to:
  - Chapter 3, Section 3.5 (shared-control architecture)
  - Chapter 3, Section 3.8 (command fusion and safety layer)
  - Chapter 4, Section 4.5 (implementation details)

The modulation is defined formally in equation (3.1) of Section 3.5.2:
    u_robot(t) = alpha(t) * u_user(t)
with the same scaling factor applied uniformly to both the linear and
the angular components of the velocity command (the source of the
trajectory-independence property verified experimentally in
Section 5.3.3).
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class CmdVelFusionNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_fusion_node')

        self.declare_parameter('cmd_input_topic', '/cmd_vel_user')
        self.declare_parameter('alpha_topic', '/alpha')
        self.declare_parameter('output_topic', '/S2R5/cmd_vel')

        # 20 Hz fixed-rate publication.  The timer is independent of the
        # arrival of messages on either input topic.  This ensures the
        # executed velocity continues to track alpha even when the
        # operator's joystick command is steady (and therefore not being
        # republished by teleop_twist_joy).  See Section 3.8.1 for the
        # rationale and Section 4.5.1 for the implementation pattern.
        self.declare_parameter('publish_rate', 20.0)

        # 500 ms command timeout.  If /cmd_vel_user has not been heard
        # for longer than this interval, the node publishes a zero
        # Twist to stop the robot.  See Section 3.8.2.
        self.declare_parameter('cmd_timeout', 0.5)

        self.declare_parameter('default_alpha', 1.0)

        # Defensive clipping bounds on alpha (Section 4.5.3).  The
        # fuzzy controller produces alpha in [0.49, 0.91] by
        # construction (Section 3.7.6), so this clipping is normally
        # not engaged.  Keeping the lower bound at 0.0 here rather
        # than at the design value 0.4 is deliberate: the fall-back
        # value 0.4 inside the fuzzy node could in principle reach
        # the fusion node, and the wider window absorbs that case
        # without producing spurious mid-range scaling.
        self.declare_parameter('alpha_min', 0.0)
        self.declare_parameter('alpha_max', 1.0)

        self.cmd_input_topic = self.get_parameter('cmd_input_topic').value
        self.alpha_topic = self.get_parameter('alpha_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.default_alpha = float(self.get_parameter('default_alpha').value)
        self.alpha_min = float(self.get_parameter('alpha_min').value)
        self.alpha_max = float(self.get_parameter('alpha_max').value)

        # Internal state: most recent command, most recent alpha, and
        # the wall-clock time at which the last command arrived (used
        # by the timeout check).
        self.latest_cmd = Twist()
        self.latest_alpha = self.default_alpha
        self.last_cmd_time = None

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_input_topic,
            self.cmd_callback,
            10
        )

        self.alpha_sub = self.create_subscription(
            Float32,
            self.alpha_topic,
            self.alpha_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_topic,
            10
        )

        # Fixed-rate timer driving the publication.  See above.
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'CmdVel fusion node started | cmd_input_topic={self.cmd_input_topic}, '
            f'alpha_topic={self.alpha_topic}, output_topic={self.output_topic}, '
            f'publish_rate={self.publish_rate}, cmd_timeout={self.cmd_timeout}'
        )

    def cmd_callback(self, msg: Twist):
        # Store-only callback.  The publication happens in the timer.
        # See Section 4.5.1 for the rationale for separating subscription
        # callbacks from the periodic publication step.
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def alpha_callback(self, msg: Float32):
        # Defensive clipping of alpha to [alpha_min, alpha_max].
        # See Section 4.5.3.
        alpha = float(msg.data)
        alpha = max(self.alpha_min, min(self.alpha_max, alpha))
        self.latest_alpha = alpha

    def is_cmd_recent(self):
        """Command-timeout check (Section 3.8.2)."""
        if self.last_cmd_time is None:
            return False
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        return dt <= self.cmd_timeout

    def timer_callback(self):
        """Fixed-rate publication of the modulated command.  Implements
        equation (3.X) of Section 3.5.2:
            u_robot = alpha * u_user
        applied uniformly to both linear and angular components."""
        out = Twist()

        if self.is_cmd_recent():
            a = self.latest_alpha

            # The same alpha is applied to every component of the Twist.
            # The architectural trajectory-independence property
            # described in Section 3.5.2 is a structural consequence
            # of this uniform application.
            out.linear.x = a * self.latest_cmd.linear.x
            out.linear.y = a * self.latest_cmd.linear.y
            out.linear.z = a * self.latest_cmd.linear.z

            out.angular.x = a * self.latest_cmd.angular.x
            out.angular.y = a * self.latest_cmd.angular.y
            out.angular.z = a * self.latest_cmd.angular.z

        # If the command is stale, an empty Twist (all zeros) is
        # published and the robot is commanded to stop (Section 3.8.2).
        self.cmd_pub.publish(out)

    def publish_stop(self):
        """Helper used by the shutdown safeguard.  Publishes a single
        zero-velocity Twist."""
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Published STOP command.')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Stopping robot...')
    finally:
        # Shutdown safeguard (Section 3.8.3 / Section 4.5.2).
        # Five consecutive zero-velocity Twist messages are published
        # to ensure that at least one reaches the Create 3 motion
        # controller even if individual messages are dropped by the
        # Wi-Fi network.  This was added in response to the runtime
        # issue described at the start of Section 4.5.2, in which a
        # terminated controller left the robot in motion until the
        # Create 3 firmware's own command-timeout engaged.
        try:
            for _ in range(5):
                node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()