#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
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

from clearpath_config.common.types.platform import Platform

from clearpath_generator_common.common import LaunchFile
from clearpath_generator_common.launch.writer import LaunchWriter
from clearpath_generator_common.launch.generator import LaunchGenerator

from clearpath_generator_gz.launch.sensors import SensorLaunch

import os


class GzLaunchGenerator(LaunchGenerator):
    GZ_TO_ROS_TWIST = '@geometry_msgs/msg/Twist[ignition.msgs.Twist'
    ROS_TO_GZ_TWIST = '@geometry_msgs/msg/Twist]ignition.msgs.Twist'
    GZ_TO_ROS_TF = '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'

    def __init__(self, setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)
        for i, arg in enumerate(self.platform_launch_file.args):
            if arg[0] == 'use_sim_time':
                self.platform_launch_file.args[i] = ('use_sim_time', 'true')

        if self.namespace in ('', '/'):
            self.robot_name = 'robot'
        else:
            self.robot_name = self.namespace + '/robot'

    def generate_sensors(self) -> None:
        sensors_service_launch_writer = LaunchWriter(self.sensors_service_launch_file)
        sensors = self.clearpath_config.sensors.get_all_sensors()

        prefix_launch_arg = LaunchFile.LaunchArg(
                'prefix',
                default_value='/world/warehouse/model/robot/link/base_link/sensor/',
                description='Ignition sensor topic prefix'
            )

        for sensor in sensors:
            if sensor.get_launch_enabled():
                sensor_launch = SensorLaunch(
                        sensor,
                        self.namespace,
                        self.sensors_launch_path,
                        self.sensors_params_path)
                sensor_writer = LaunchWriter(sensor_launch.get_launch_file())
                # Add sensor bridge and tf nodes
                sensor_writer.add_node(sensor_launch.get_gz_bridge_node())
                sensor_writer.add_node(sensor_launch.get_static_tf_node())
                sensor_writer.declare_launch_arg(prefix_launch_arg)
                # Generate sensor launch file
                sensor_writer.generate_file()
                # Add sensor to top level sensors launch file
                sensors_service_launch_writer.add_launch_file(sensor_launch.get_launch_file())

        sensors_service_launch_writer.declare_launch_arg(prefix_launch_arg)
        sensors_service_launch_writer.generate_file()

    def generate_platform(self) -> None:
        platform_service_launch_writer = LaunchWriter(self.platform_service_launch_file)
        platform_service_launch_writer.add_launch_file(self.platform_launch_file)

        # cmd_vel bridge
        if self.namespace in ('', '/'):
            cmd_vel_bridge_arg = '/cmd_vel' + self.GZ_TO_ROS_TWIST
            cmd_vel_bridge_remap = ('/cmd_vel', 'cmd_vel')
        else:
            cmd_vel_bridge_arg = self.namespace + '/cmd_vel' + self.GZ_TO_ROS_TWIST
            cmd_vel_bridge_remap = (self.namespace + '/cmd_vel', 'cmd_vel')

        cmd_vel_robot_bridge_arg = '/model/' + self.robot_name + '/cmd_vel' + self.ROS_TO_GZ_TWIST
        cmd_vel_robot_bridge_remap = (
            '/model/' + self.robot_name + '/cmd_vel',
            'platform/cmd_vel_unstamped'
          )

        cmd_vel_node = LaunchFile.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            namespace=self.namespace,
            parameters=[{'use_sim_time': True}],
            arguments=[
                cmd_vel_bridge_arg,
                cmd_vel_robot_bridge_arg
            ],
            remappings=[
                cmd_vel_bridge_remap,
                cmd_vel_robot_bridge_remap
            ])

        # odom to base_link tf bridge
        odom_base_node = LaunchFile.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_base_tf_bridge',
            namespace=self.namespace,
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/model/' + self.robot_name + '/tf' + self.GZ_TO_ROS_TF
            ],
            remappings=[
                ('/model/' + self.robot_name + '/tf', 'tf')
            ])

        # Prefix launch arg
        prefix_launch_arg = LaunchFile.LaunchArg(
            'prefix',
            default_value='/world/warehouse/model/robot/link/base_link/sensor/',
            description='Ignition sensor topic prefix'
        )
        prefix_variable = LaunchFile.Variable('prefix')

        # Builtin IMU bridge
        imu_0_bridge_node = LaunchFile.Node(
          name='imu_0_gz_bridge',
          package='ros_gz_bridge',
          executable='parameter_bridge',
          namespace=self.namespace,
          parameters=[{'use_sim_time': True}],
          arguments=[[
            prefix_variable,
            'imu_link/imu' + SensorLaunch.BaseLaunch.GZ_TO_ROS_IMU
          ]],
          remappings=[
            ([prefix_variable, 'imu_link/imu'],
             'platform/sensors/imu_0/data_raw')
          ]
        )

        # IMU static tf
        imu_0_static_tf_node = LaunchFile.get_static_tf_node(
          name='imu_0',
          namespace=self.namespace,
          parent_link='imu_link',
          child_link=self.robot_name + '/base_link/imu_link',
          use_sim_time=True
        )

        # IMU filter
        imu_filter_arg = LaunchFile.LaunchArg(
            'imu_filter',
            default_value=os.path.join(self.platform_params_path, 'imu_filter.yaml')
        )
        imu_filter_variable = LaunchFile.Variable('imu_filter')

        imu_filter_node = LaunchFile.Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            namespace=self.namespace,
            parameters=[imu_filter_variable],
            remappings=[
              ('imu/data_raw', 'platform/sensors/imu_0/data_raw'),
              ('imu/mag', 'platform/sensors/imu_0/magnetic_field'),
              ('imu/data', 'platform/sensors/imu_0/data'),
              ('/tf', 'tf'),
            ],
        )

        # GPS bridge
        gps_0_bridge_node = LaunchFile.Node(
          name='gps_0_gz_bridge',
          package='ros_gz_bridge',
          executable='parameter_bridge',
          namespace=self.namespace,
          parameters=[{'use_sim_time': True}],
          arguments=[[
            prefix_variable,
            'navsat_link/navsat' + SensorLaunch.BaseLaunch.GZ_TO_ROS_NAVSAT
          ]],
          remappings=[
            ([prefix_variable,
              'navsat_link/navsat'],
             'platform/sensors/gps_0/navsat')
          ]
        )

        # GPS static tf
        gps_0_static_tf_node = LaunchFile.get_static_tf_node(
          name='gps_0',
          namespace=self.namespace,
          parent_link='navsat_link',
          child_link=self.robot_name + '/base_link/navsat_link',
          use_sim_time=True
        )

        # Static transform from <namespace>/odom to odom
        # See https://github.com/ros-controls/ros2_controllers/pull/533
        tf_namespaced_odom_publisher = LaunchFile.get_static_tf_node(
            name='namespaced_odom',
            namespace=self.namespace,
            parent_link='odom',
            child_link=self.namespace + '/odom',
            use_sim_time=True
        )

        # Static transform from <namespace>/base_link to base_link

        tf_namespaced_base_link_publisher = LaunchFile.get_static_tf_node(
            name='namespaced_base_link',
            namespace=self.namespace,
            parent_link=self.namespace + '/base_link',
            child_link='base_link',
            use_sim_time=True
        )

        # Common Nodes
        platform_service_launch_writer.add_node(cmd_vel_node)
        platform_service_launch_writer.add_node(odom_base_node)

        if self.namespace not in ('', '/'):
            platform_service_launch_writer.add_node(tf_namespaced_odom_publisher)
            platform_service_launch_writer.add_node(tf_namespaced_base_link_publisher)

        # J100 Nodes
        if self.platform_model == Platform.J100:
            platform_service_launch_writer.declare_launch_arg(prefix_launch_arg)
            platform_service_launch_writer.declare_launch_arg(imu_filter_arg)
            platform_service_launch_writer.add_node(imu_0_bridge_node)
            platform_service_launch_writer.add_node(imu_0_static_tf_node)
            platform_service_launch_writer.add_node(imu_filter_node)
            platform_service_launch_writer.add_node(gps_0_bridge_node)
            platform_service_launch_writer.add_node(gps_0_static_tf_node)

        platform_service_launch_writer.generate_file()
