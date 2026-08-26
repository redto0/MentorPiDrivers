"""IMU filter chain: optional static calibration, then a complementary filter.

Chain, when the calibration file is present:
    /ros_robot_controller/imu_raw -> apply_calib -> imu_corrected -> filter -> imu
Without it, apply_calib is skipped and the filter runs straight off the raw topic.

Changes from Hiwonder upstream:
  - the calibration YAML lives in this package rather than the `calibration`
    package, which this fork does not ship
  - a missing calibration file degrades to the uncalibrated chain instead of
    raising FileNotFoundError and aborting the whole launch
  - executable is apply_calib_node, which is what the ROS 2 port of imu_calib
    actually builds (upstream called apply_calib, a ROS 1 name)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

RAW_TOPIC = '/ros_robot_controller/imu_raw'


def generate_launch_description():
    calib_file = os.path.join(get_package_share_directory('peripherals'), 'config', 'imu_calib.yaml')
    have_calib = os.path.exists(calib_file)

    nodes = []
    filter_input = RAW_TOPIC

    if have_calib:
        nodes.append(Node(
            package='imu_calib',
            executable='apply_calib_node',
            name='imu_calib',
            output='screen',
            parameters=[{'calib_file': calib_file}],
            remappings=[
                ('raw', RAW_TOPIC),
                ('corrected', 'imu_corrected'),
            ],
        ))
        filter_input = 'imu_corrected'

    nodes.append(Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'do_bias_estimation': True,
            'do_adaptive_gain': True,
            'publish_debug_topics': True,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/imu/data_raw', filter_input),
            ('imu/data', 'imu'),
        ],
    ))

    # Upstream delays 5 s so the serial link to the STM32 is up first.
    return LaunchDescription([TimerAction(period=5.0, actions=nodes)])
