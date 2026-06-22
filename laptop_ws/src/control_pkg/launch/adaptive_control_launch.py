"""
Launch configuration for the adaptive control system.

Starts the three custom ROS 2 nodes hosted on the development laptop:
    vibration_feature_node
    fuzzy_alpha_node
    cmd_vel_fusion_node

The fourth custom node (imu_reader_node) and the standard ROS 2 nodes
(joy_node, teleop_twist_joy) run on the Raspberry Pi and are started
separately, as documented in Section 4.7.2.  The Create 3 hosts no
user nodes (Section 3.4.2).

The fuzzy_enabled launch argument is the single point of control that
switches between the fuzzy-ON and fuzzy-OFF experimental conditions
defined in Chapter 5.  See the invocation examples in Section 4.7.1:
    ros2 launch ... fuzzy_enabled:=true    # fuzzy-ON
    ros2 launch ... fuzzy_enabled:=false   # fuzzy-OFF

This launch configuration corresponds to:
  - Chapter 4, Section 4.7 (launch configuration and experimental workflow)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    feature_topic = LaunchConfiguration('feature_topic')
    alpha_topic = LaunchConfiguration('alpha_topic')
    user_cmd_topic = LaunchConfiguration('user_cmd_topic')
    output_topic = LaunchConfiguration('output_topic')
    window_size = LaunchConfiguration('window_size')
    fuzzy_enabled = LaunchConfiguration('fuzzy_enabled')
    record_bag = LaunchConfiguration('record_bag')
    bag_name = LaunchConfiguration('bag_name')

    # Optional bag recording.  Records the topics used in the analysis
    # of Chapter 5: /imu/raw, /vibration_feature, /cmd_vel_user,
    # the modulated /S2R5/cmd_vel, and /odom.

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/imu/raw',
            feature_topic,
            user_cmd_topic,
            output_topic,
            '/odom',
            '-o', bag_name
        ],
        output='screen',
        condition=IfCondition(record_bag)
    )

    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/imu/raw'),
        DeclareLaunchArgument('feature_topic', default_value='/vibration_feature'),
        DeclareLaunchArgument('alpha_topic', default_value='/alpha'),
        DeclareLaunchArgument('user_cmd_topic', default_value='/cmd_vel_user'),
        DeclareLaunchArgument('output_topic', default_value='/S2R5/cmd_vel'),

        # Final tuned window length W = 70 (Section 3.6.3, Figure 3.6).
        DeclareLaunchArgument('window_size', default_value='70'),

        # Default fuzzy-ON.  Set to 'false' at launch time to produce
        # the fuzzy-OFF baseline experimental condition (Chapter 5).
        DeclareLaunchArgument('fuzzy_enabled', default_value='true'),

        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_name', default_value='test_run'),

        # vibration_feature_node: see Section 3.6 and Section 4.3.
        Node(
            package='control_pkg',
            executable='vibration_feature_node',
            name='vibration_feature_node',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': feature_topic,
                'window_size': window_size,
                'convert_to_g': True
            }]
        ),

        # fuzzy_alpha_node: see Section 3.7 and Section 4.4.
        Node(
            package='control_pkg',
            executable='fuzzy_alpha_node',
            name='fuzzy_alpha_node',
            output='screen',
            parameters=[{
                'input_topic': feature_topic,
                'output_topic': alpha_topic,
                'fuzzy_enabled': fuzzy_enabled
            }]
        ),

        # cmd_vel_fusion_node: see Section 3.5, Section 3.8 and Section 4.5.
        # publish_rate, cmd_timeout, default_alpha, alpha_min, alpha_max
        # are the values documented in Section 4.5 and used for all
        # experimental runs of Chapter 5.
        Node(
            package='control_pkg',
            executable='cmd_vel_fusion_node',
            name='cmd_vel_fusion_node',
            output='screen',
            parameters=[{
                'cmd_input_topic': user_cmd_topic,
                'alpha_topic': alpha_topic,
                'output_topic': output_topic,
                'publish_rate': 20.0,
                'cmd_timeout': 0.5,
                'default_alpha': 1.0,
                'alpha_min': 0.0,
                'alpha_max': 1.0
            }]
        ),

        bag_record
    ])