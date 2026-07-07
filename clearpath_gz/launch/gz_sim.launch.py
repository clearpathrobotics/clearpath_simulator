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
import tempfile

from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    DeclareLaunchArgument('auto_start', default_value='true',
                          choices=['true', 'false'],
                          description='Auto-start Gazebo simulation'),
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
]


def gz_launch(context, *args, **kwargs):

    # Directories
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    static_gui_config = os.path.join(pkg_clearpath_gz, 'config', 'gui.config')

    # Derive teleop topic from robot.yaml namespace
    setup_path = LaunchConfiguration('setup_path').perform(context)
    robot_yaml = os.path.join(setup_path, 'robot.yaml')
    if os.path.isfile(robot_yaml):
        clearpath_config = ClearpathConfig(robot_yaml)
        namespace = clearpath_config.system.namespace
        topic = f'/{namespace}/cmd_vel' if namespace not in ('', '/') else '/cmd_vel'
        with open(static_gui_config) as f:
            content = f.read()
        content = content.replace(
            '<plugin filename="Teleop">',
            f'<plugin filename="Teleop">\n    <topic>{topic}</topic>'
        )
        tmp = tempfile.NamedTemporaryFile(
            suffix='.config', delete=False, mode='w')
        tmp.write(content)
        tmp.close()
        gui_config = tmp.name
    else:
        gui_config = static_gui_config

    auto_start_option = ''
    auto_start = LaunchConfiguration('auto_start').perform(context)
    if (auto_start == 'true'):
        auto_start_option = ' -r'

    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                         '.sdf',
                         auto_start_option,
                         ' -v 4',
                         ' --gui-config ',
                         gui_config])
        ]
    )

    return [gz_sim]


def generate_launch_description():

    # Directories
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')

    # Determine all ros packages that are sourced
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # Set ignition resource path to include all sourced ros packages
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_clearpath_gz, 'worlds') + ':',
            os.path.join(pkg_clearpath_gz, 'meshes') + ':',
            ':' + ':'.join(packages_paths)])

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(OpaqueFunction(function=gz_launch))
    ld.add_action(clock_bridge)
    return ld
