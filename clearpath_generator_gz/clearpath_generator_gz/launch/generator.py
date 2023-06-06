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

from clearpath_config.platform.platform import Platform

from clearpath_generator_common.common import LaunchFile
from clearpath_generator_common.launch.writer import LaunchWriter
from clearpath_generator_common.launch.generator import LaunchGenerator

from clearpath_generator_gz.launch.sensors import SensorLaunch

import os


class GzLaunchGenerator(LaunchGenerator):
    def __init__(self, setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)
        self.platform_launch_file.args['use_sim_time'] = 'true'

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

        if self.platform_model == Platform.J100:
            # Prefix launch arg
            prefix_launch_arg = LaunchFile.LaunchArg(
                'prefix',
                default_value='/world/warehouse/model/robot/link/base_link/sensor/',
                description='Ignition sensor topic prefix'
            )
            platform_service_launch_writer.declare_launch_arg(prefix_launch_arg)
            # IMU bridge
            platform_service_launch_writer.add_node(
              LaunchFile.Node(
                name='imu_0_gz_bridge',
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=self.namespace,
                parameters=[{'use_sim_time': True}],
                arguments=[[
                  'prefix',
                  '\'imu_link/imu' +
                  SensorLaunch.BaseLaunch.GZ_TO_ROS_IMU + '\''
                ]],
                remappings=[
                  (['prefix',
                    '\'imu_link/imu\''],
                    '\'platform/sensors/imu_0/data_raw\'')
                ]
              )
            )
            # IMU static tf
            platform_service_launch_writer.add_node(
                LaunchFile.Node(
                  name='imu_0_static_tf',
                  package='tf2_ros',
                  executable='static_transform_publisher',
                  namespace=self.namespace,
                  parameters=[{'use_sim_time': True}],
                  arguments=[
                      '0', '0', '0', '0', '0', '0.0',
                      'imu_link',
                      self.namespace + '/robot/base_link/imu_link'
                  ],
                  remappings=[
                      ('\'/tf\'', '\'tf\''),
                      ('\'/tf_static\'', '\'tf_static\''),
                  ])
            )
            # GPS bridge
            platform_service_launch_writer.add_node(
              LaunchFile.Node(
                name='gps_0_gz_bridge',
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=self.namespace,
                parameters=[{'use_sim_time': True}],
                arguments=[[
                  'prefix',
                  '\'navsat_link/navsat' +
                  SensorLaunch.BaseLaunch.GZ_TO_ROS_NAVSAT + '\''
                ]],
                remappings=[
                  (['prefix',
                    '\'navsat_link/navsat\''],
                    '\'platform/sensors/gps_0/navsat\'')
                ]
              )
            )
            # GPS static tf
            platform_service_launch_writer.add_node(
                LaunchFile.Node(
                  name='gps_0_static_tf',
                  package='tf2_ros',
                  executable='static_transform_publisher',
                  namespace=self.namespace,
                  parameters=[{'use_sim_time': True}],
                  arguments=[
                      '0', '0', '0', '0', '0', '0.0',
                      'navsat_link',
                      self.namespace + '/robot/base_link/navsat_link'
                  ],
                  remappings=[
                      ('\'/tf\'', '\'tf\''),
                      ('\'/tf_static\'', '\'tf_static\''),
                  ])
            )

            # IMU filter
            imu_filter_config = LaunchFile.LaunchArg(
                'imu_filter',
                default_value=os.path.join(self.platform_params_path, 'imu_filter.yaml')
            )
            platform_service_launch_writer.declare_launch_arg(imu_filter_config)

            imu_filter_node = LaunchFile.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter_node',
                namespace=self.namespace,
                parameters=['imu_filter'],
                remappings=[
                  ('\'imu/data_raw\'', '\'platform/sensors/imu_0/data_raw\''),
                  ('\'imu/mag\'', '\'platform/sensors/imu_0/magnetic_field\''),
                  ('\'imu/data\'', '\'platform/sensors/imu_0/data\''),
                  ('\'/tf\'', '\'tf\''),
                ],
            )
            platform_service_launch_writer.add_node(imu_filter_node)

        # Static transform from <namespace>/odom to odom
        # See https://github.com/ros-controls/ros2_controllers/pull/533
        tf_namespaced_odom_publisher = LaunchFile.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_namespaced_odom_publisher',
            namespace=self.namespace,
            arguments=['0', '0', '0',
                       '0', '0', '0',
                       'odom', self.namespace + '/odom'],
            remappings=[
                ('\'/tf\'', '\'tf\''),
                ('\'/tf_static\'', '\'tf_static\''),
            ],
        )

        # Static transform from <namespace>/base_link to base_link
        tf_namespaced_base_link_publisher = LaunchFile.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_namespaced_base_link_publisher',
            namespace=self.namespace,
            arguments=['0', '0', '0',
                       '0', '0', '0',
                       self.namespace + '/base_link', 'base_link'],
            remappings=[
                ('\'/tf\'', '\'tf\''),
                ('\'/tf_static\'', '\'tf_static\''),
            ],
        )

        if self.namespace:
            platform_service_launch_writer.add_node(tf_namespaced_odom_publisher)
            platform_service_launch_writer.add_node(tf_namespaced_base_link_publisher)

        platform_service_launch_writer.generate_file()
