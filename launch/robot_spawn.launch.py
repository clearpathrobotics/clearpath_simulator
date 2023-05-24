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


from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to launch localization'),
    DeclareLaunchArgument('slam', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to launch SLAM'),
    DeclareLaunchArgument('nav2', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to launch Nav2'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():

    # Directories
    pkg_clearpath_simulator = get_package_share_directory(
        'clearpath_simulator')
    pkg_clearpath_platform_description = get_package_share_directory(
        'clearpath_platform_description')
    pkg_clearpath_viz = get_package_share_directory(
        'clearpath_viz')
    pkg_clearpath_control = get_package_share_directory(
        'clearpath_control'
    )

    # Paths
    rviz_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_model.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_clearpath_platform_description, 'launch', 'description.launch.py'])
    ros_gz_bridge_launch = PathJoinSubstitution(
        [pkg_clearpath_simulator, 'launch', 'ros_gz_bridge.launch.py'])

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    robot_name = GetNamespacedName(namespace, 'robot')

    # Spawn robot slightly clsoer to the floor to reduce the drop
    # Ensures robot remains properly docked after the drop
    z_robot = OffsetParser(z, 0.05)

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments=[
              ('use_sim_time', use_sim_time)
            ]
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z_robot,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),

        # ros_gz_bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ros_gz_bridge_launch]),
            launch_arguments=[
              ('use_sim_time', use_sim_time),
              ('robot_name', 'robot'),
              ('namespace', namespace)
            ]
        ),

        # Launch clearpath_control/control.launch.py which is just robot_localization.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
              [pkg_clearpath_control, 'launch', 'control.launch.py'])),
            launch_arguments=[('is_sim', use_sim_time)]
        ),

        # Launch clearpath_control/teleop_base.launch.py which is various ways to tele-op
        # the robot but does not include the joystick. Also, has a twist mux.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
              [pkg_clearpath_control, 'launch', 'teleop_base.launch.py'])),
        ),
    ])

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
    ld.add_action(spawn_robot_group_action)
    ld.add_action(rviz)
    return ld
