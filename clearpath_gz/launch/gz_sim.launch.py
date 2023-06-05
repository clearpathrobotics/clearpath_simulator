# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
]


def generate_launch_description():

    # Directories
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')
    pkg_clearpath_platform_description = get_package_share_directory(
        'clearpath_platform_description')
    pkg_clearpath_sensors_description = get_package_share_directory(
        'clearpath_sensors_description')
    pkg_clearpath_mounts_description = get_package_share_directory(
        'clearpath_mounts_description')
    pkg_velodyne_description = get_package_share_directory(
        'velodyne_description')
    pkg_realsense2_description = get_package_share_directory(
        'realsense2_description')
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    # Set ignition resource path
    gz_sim_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_clearpath_gz, 'worlds'), ':' +
            str(Path(pkg_clearpath_platform_description).parent.resolve()), ':' +
            str(Path(pkg_clearpath_sensors_description).parent.resolve()), ':' +
            str(Path(pkg_clearpath_mounts_description).parent.resolve()), ':' +
            str(Path(pkg_velodyne_description).parent.resolve()), ':' +
            str(Path(pkg_realsense2_description).parent.resolve())])

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    gui_config = PathJoinSubstitution(
        [pkg_clearpath_gz, 'config', 'gui.config'])

    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                         '.sdf',
                         ' -v 4',
                         ' --gui-config ',
                         gui_config])
        ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                          '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
                        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(gz_sim)
    ld.add_action(clock_bridge)
    return ld
