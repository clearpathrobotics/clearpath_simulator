# Copyright 2021 Clearpath Robotics, Inc.
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


from launch import (
    LaunchContext,
    LaunchDescription,
    SomeSubstitutionsType,
    Substitution
)
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from typing import Union


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.15',
                 description='z component of the robot pose.'))


class GetNamespacedName(Substitution):
    def __init__(
            self,
            namespace: Union[SomeSubstitutionsType, str],
            name: Union[SomeSubstitutionsType, str]
    ) -> None:
        self.__namespace = namespace
        self.__name = name

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        if isinstance(self.__namespace, Substitution):
            namespace = str(self.__namespace.perform(context))
        else:
            namespace = str(self.__namespace)

        if isinstance(self.__name, Substitution):
            name = str(self.__name.perform(context))
        else:
            name = str(self.__name)

        if namespace == '':
            namespaced_name = name
        else:
            namespaced_name = namespace + '/' + name
        return namespaced_name


def generate_launch_description():

    # Directories
    pkg_clearpath_gz = FindPackageShare('clearpath_gz')
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    launch_arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
        description='Clearpath setup path')

    # Launch configurations
    setup_path = LaunchConfiguration('setup_path')
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = GetNamespacedName(namespace, 'robot')

    # Paths
    rviz_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_robot.launch.py'])
    ros_gz_bridge_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'ros_gz_bridge.launch.py'])
    launch_file_platform_service = PathJoinSubstitution([
        setup_path, 'platform/launch', 'platform-service.launch.py'])
    launch_file_sensors_service = PathJoinSubstitution([
        setup_path, 'sensors/launch', 'sensors-service.launch.py'])

    group_action_spawn_robot = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_platform_service])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_sensors_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),

        # ros_gz_bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ros_gz_bridge_launch]),
            launch_arguments=[
              ('use_sim_time', use_sim_time),
              ('robot_name', robot_name),
              ('namespace', namespace)
            ]
        ),
    ])

    node_generate_description = Node(
        package='clearpath_generator_common',
        executable='generate_description',
        name='generate_description',
        output='screen',
        arguments=['-s', setup_path]
    )

    node_generate_launch = Node(
        package='clearpath_generator_gz',
        executable='generate_launch',
        name='generate_launch',
        output='screen',
        arguments=['-s', setup_path]
    )

    node_generate_param = Node(
        package='clearpath_generator_gz',
        executable='generate_param',
        name='generate_launch',
        output='screen',
        arguments=['-s', setup_path]
    )

    event_generate_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_description,
            on_exit=[node_generate_launch]
        )
    )

    event_generate_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_launch,
            on_exit=[node_generate_param]
        )
    )

    event_generate_param = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_param,
            on_exit=[group_action_spawn_robot]
        )
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_arg_setup_path)
    ld.add_action(node_generate_description)
    ld.add_action(event_generate_description)
    ld.add_action(event_generate_launch)
    ld.add_action(event_generate_param)
    ld.add_action(rviz)
    return ld
