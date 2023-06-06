# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='robot',
                          description='Ignition model name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')

    # cmd_vel bridge
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            [namespace,
             '/cmd_vel' + '@geometry_msgs/msg/Twist' + '[ignition.msgs.Twist'],
            ['/model/', robot_name, '/cmd_vel' +
             '@geometry_msgs/msg/Twist' +
             ']ignition.msgs.Twist']
        ],
        remappings=[
            ([namespace, '/cmd_vel'], 'cmd_vel'),
            (['/model/', robot_name, '/cmd_vel'],
             'platform/cmd_vel_unstamped')
        ])

    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_base_tf_bridge',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/model/', robot_name, '/tf' +
            '@tf2_msgs/msg/TFMessage' +
            '[ignition.msgs.Pose_V']
        ],
        remappings=[
            (['/model/', robot_name, '/tf'], 'tf')
        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(odom_base_tf_bridge)
    return ld
