# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
"""Shared factory functions for Gazebo launch components."""
import os

from clearpath_generator_common.common import LaunchFile


GZ_TO_ROS_TWIST = '@geometry_msgs/msg/TwistStamped[gz.msgs.Twist'
ROS_TO_GZ_TWIST = '@geometry_msgs/msg/TwistStamped]gz.msgs.Twist'
GZ_TO_ROS_TF = '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'


def make_cmd_vel_node(namespace: str, robot_name: str) -> LaunchFile.Node:
    """Build the ros_gz_bridge `parameter_bridge` for cmd_vel topics."""
    if namespace in ('', '/'):
        cmd_vel_bridge_arg = '/cmd_vel' + GZ_TO_ROS_TWIST
        cmd_vel_bridge_remap = ('/cmd_vel', 'cmd_vel')
    else:
        cmd_vel_bridge_arg = namespace + '/cmd_vel' + GZ_TO_ROS_TWIST
        cmd_vel_bridge_remap = (namespace + '/cmd_vel', 'cmd_vel')

    cmd_vel_robot_bridge_arg = '/model/' + robot_name + '/cmd_vel' + ROS_TO_GZ_TWIST
    cmd_vel_robot_bridge_remap = (
        '/model/' + robot_name + '/cmd_vel',
        'platform/cmd_vel'
    )

    return LaunchFile.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        namespace=namespace,
        parameters=[{'use_sim_time': True}],
        arguments=[
            cmd_vel_bridge_arg,
            cmd_vel_robot_bridge_arg
        ],
        remappings=[
            cmd_vel_bridge_remap,
            cmd_vel_robot_bridge_remap
        ])


def make_odom_base_node(namespace: str, robot_name: str) -> LaunchFile.Node:
    """Build the ros_gz_bridge `parameter_bridge` that publishes odom->base TF."""
    return LaunchFile.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_base_tf_bridge',
        namespace=namespace,
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/model/' + robot_name + '/tf' + GZ_TO_ROS_TF
        ],
        remappings=[
            ('/model/' + robot_name + '/tf', 'tf')
        ])


def make_imu_0_bridge_node(namespace: str, sensors_params_path: str) -> LaunchFile.Node:
    """Build the ros_gz_bridge for the built-in IMU configured by `imu_0.yaml`."""
    return LaunchFile.Node(
        name='imu_0_gz_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(sensors_params_path, 'imu_0.yaml')
        }]
    )


def make_imu_filter_arg(platform_params_path: str) -> LaunchFile.LaunchArg:
    """Build the `imu_filter` LaunchArg pointing at the platform IMU filter config."""
    return LaunchFile.LaunchArg(
        'imu_filter',
        default_value=os.path.join(platform_params_path, 'imu_filter.yaml')
    )


def make_imu_filter_node(namespace: str) -> LaunchFile.Node:
    """Build the `imu_filter_madgwick_node` that fuses the bridged IMU data."""
    return LaunchFile.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
        namespace=namespace,
        parameters=[LaunchFile.Variable('imu_filter')],
        remappings=[
            ('imu/data_raw', 'sensors/imu_0/data_raw'),
            ('imu/mag', 'sensors/imu_0/magnetic_field'),
            ('imu/data', 'sensors/imu_0/data'),
            ('/tf', 'tf'),
        ],
    )


def make_gps_0_bridge_node(namespace: str, sensors_params_path: str) -> LaunchFile.Node:
    """Build the ros_gz_bridge for the built-in GPS configured by `gps_0.yaml`."""
    return LaunchFile.Node(
        name='gps_0_gz_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(sensors_params_path, 'gps_0.yaml')
        }]
    )
