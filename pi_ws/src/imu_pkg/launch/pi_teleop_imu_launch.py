"""
Launch configuration for the Raspberry Pi side of the system.

Starts the three nodes hosted on the Pi:
    imu_reader_node    -- custom; reads MPU-9250 over I2C
    joy_node           -- standard ROS 2; reads PS4 controller over Bluetooth
    teleop_twist_joy   -- standard ROS 2; maps joystick to Twist commands

The teleop_twist_joy output is remapped from its default /cmd_vel topic
to /cmd_vel_user, the topic consumed by the laptop's
cmd_vel_fusion_node.  The PS4 axis mapping and the maximum velocity
scaling are loaded from the config file ps4_teleop.yaml in the
imu_pkg share directory.

The laptop-side nodes (vibration_feature_node, fuzzy_alpha_node,
cmd_vel_fusion_node) are started separately by
adaptive_control_launch.py, as documented in Section 4.7.

This launch configuration corresponds to:
  - Chapter 3, Section 3.4.2 (host mapping; the Pi hosts the
    hardware-facing nodes)
  - Chapter 4, Section 4.7.2 (pre-run startup sequence)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # PS4 controller axis-to-velocity mapping configuration.  The
    # left stick is mapped to linear velocity, the right stick to
    # angular velocity; the maximum scaling factors are configured to
    # give nominal 0.20 m/s linear and ~0.80 rad/s angular at full
    # stick deflection (Section 3.2.4).
    ps4_config = PathJoinSubstitution([
        FindPackageShare('imu_pkg'),
        'config',
        'ps4_teleop.yaml'
    ])

    return LaunchDescription([

        # imu_reader_node (custom).  Reads MPU-9250 accelerometer over
        # I2C at 100 Hz and publishes on /imu/raw.  See Section 3.4.1
        # (decomposition) and Section 4.2 (implementation, including
        # the bias and DLPF disclosures of Sections 4.2.2 and 4.2.3).
        Node(
            package='imu_pkg',
            executable='imu_reader_node',
            name='imu_reader_node',
            output='screen'
        ),

        # joy_node (standard ROS 2 package).  Reads the paired PS4
        # DualShock 4 controller via the Linux joystick interface and
        # publishes raw button and axis state as a sensor_msgs/Joy
        # message.  Used without modification (Section 3.4.1).
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # teleop_twist_joy (standard ROS 2 package).  Converts the
        # sensor_msgs/Joy stream from joy_node into a
        # geometry_msgs/Twist velocity command, scaled by the maximum
        # velocity values configured in ps4_teleop.yaml.  The output
        # topic is remapped from the default /cmd_vel to /cmd_vel_user
        # so that the cmd_vel_fusion_node on the laptop receives the
        # operator command on the topic name it expects.  Topic and
        # remapping are described in Section 3.4 and the laptop-side
        # consumer in Section 3.8.
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[ps4_config],
            remappings=[
                ('/cmd_vel', '/cmd_vel_user')
            ]
        ),
    ])