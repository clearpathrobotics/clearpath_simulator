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

from clearpath_config.sensors.types.cameras import (
    AxisCamera,
    BaseCamera,
    IntelRealsense,
    StereolabsZed
)
from clearpath_config.sensors.types.sensor import BaseSensor
from clearpath_generator_common.common import LaunchFile, ParamFile
from clearpath_generator_common.launch.writer import LaunchWriter


class SensorLaunch():
    TOPIC_NAMESPACE = 'sensors/'

    # Launch arguments
    PARAMETERS = 'parameters'
    NAMESPACE = 'namespace'

    # gz to ros bridge parameters
    GZ_TO_ROS_LASERSCAN = '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
    GZ_TO_ROS_POINTCLOUD = '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
    GZ_TO_ROS_IMAGE = '@sensor_msgs/msg/Image[gz.msgs.Image'
    GZ_TO_ROS_CAMERA_INFO = '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
    GZ_TO_ROS_IMU = '@sensor_msgs/msg/Imu[gz.msgs.IMU'
    GZ_TO_ROS_NAVSAT = '@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'
    GZ_TO_ROS_JOINTSTATE = '@sensor_msgs/msg/JointState[gz.msgs.Model'

    ROS_TO_GZ_FLOAT = '@std_msgs/msg/Float64]gz.msgs.Double'

    RGBD_CAMERAS = [
        IntelRealsense.SENSOR_MODEL,
        StereolabsZed.SENSOR_MODEL
    ]

    PTZ_CAMERAS = [
        AxisCamera.SENSOR_MODEL
    ]

    def __init__(
          self,
          sensor: BaseSensor,
          robot_namespace: str,
          launch_path: str,
          param_path: str) -> None:
        self.sensor = sensor
        self._robot_namespace = robot_namespace
        self.parameters = ParamFile(self.name, path=param_path)
        self.prefix_launch_arg = LaunchFile.LaunchArg('prefix')

        # Generated
        self.launch_file = LaunchFile(
            self.name,
            path=launch_path)

        self.static_tf_node = LaunchFile.get_static_tf_node(
            name=self.name,
            namespace=self._robot_namespace,
            parent_link=self.name + '_link',
            child_link=self.robot_name + '/base_link/' + self.name,
            use_sim_time=True
        )

        self.gz_bridge_node = LaunchFile.Node(
            name=self.name + '_gz_bridge',
            namespace=self.namespace,
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'use_sim_time': True,
                'config_file': self.parameters.full_path
            }]
        )

        self.extra_gz_nodes = []

        # Cameras
        if (
            self.sensor.SENSOR_TYPE == BaseCamera.SENSOR_TYPE and
            self.sensor.SENSOR_MODEL not in self.PTZ_CAMERAS
        ):
            image_ns = '/' + self.namespace + self.name
            image_topic = image_ns + '/image'

            image_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_image_bridge',
                namespace=self.namespace,
                package='ros_gz_image',
                executable='image_bridge',
                parameters=[{
                    'use_sim_time': True,
                }],
                arguments=[image_topic],
                remappings=[
                    (image_topic, image_ns + '/color/image'),
                    (image_topic + '/compressed', image_ns + '/color/compressed'),
                    (image_topic + '/compressedDepth', image_ns + '/color/compressedDepth'),
                    (image_topic + '/theora', image_ns + '/color/theora'),
                ]
            )
            self.extra_gz_nodes.append(image_bridge_node)

        if self.sensor.SENSOR_MODEL in self.RGBD_CAMERAS:
            depth_ns = '/' + self.namespace + self.name
            depth_topic = depth_ns + '/depth_image'

            depth_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_depth_bridge',
                namespace=self.namespace,
                package='ros_gz_image',
                executable='image_bridge',
                parameters=[{
                    'use_sim_time': True,
                }],
                arguments=[depth_topic],
                remappings=[
                    (depth_topic, depth_ns + '/depth/image'),
                    (depth_topic + '/compressed', depth_ns + '/depth/compressed'),
                    (depth_topic + '/compressedDepth', depth_ns + '/depth/compressedDepth'),
                    (depth_topic + '/theora', depth_ns + '/depth/theora'),
                ]
            )
            self.extra_gz_nodes.append(depth_bridge_node)

        if self.sensor.SENSOR_MODEL in self.PTZ_CAMERAS:
            image_ns = '/' + self.namespace + self.name
            image_topic = image_ns + '/image'

            image_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_image_bridge',
                namespace=self.namespace,
                package='ros_gz_image',
                executable='image_bridge',
                parameters=[{
                    'use_sim_time': True,
                }],
                arguments=[image_topic],
                remappings=[
                    (image_topic, image_ns + '/_/image_raw'),
                    (image_topic + '/compressed', image_ns + '/_/compressed'),
                    (image_topic + '/compressedDepth', image_ns + '/_/compressedDepth'),
                    (image_topic + '/theora', image_ns + '/_/theora'),
                ]
            )
            self.extra_gz_nodes.append(image_bridge_node)

            cmd_ns = '/' + self.namespace + self.name
            cmd_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_cmd_bridge',
                namespace=self.namespace,
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    cmd_ns + '/cmd_pan_vel' + self.ROS_TO_GZ_FLOAT,
                    cmd_ns + '/cmd_tilt_vel' + self.ROS_TO_GZ_FLOAT,
                    cmd_ns + '/pan_joint_state' + self.GZ_TO_ROS_JOINTSTATE,
                    cmd_ns + '/tilt_joint_state' + self.GZ_TO_ROS_JOINTSTATE,
                ],
                remappings=[
                    (cmd_ns + '/pan_joint_state', '/' + self._robot_namespace + '/platform/joint_states'),  # noqa:E501
                    (cmd_ns + '/tilt_joint_state', '/' + self._robot_namespace + '/platform/joint_states'),  # noqa:E501
                    (cmd_ns + '/cmd_pan_vel', cmd_ns + '/_/cmd_pan_vel'),
                    (cmd_ns + '/cmd_tilt_vel', cmd_ns + '/_/cmd_tilt_vel'),
                ],
            )
            self.extra_gz_nodes.append(cmd_bridge_node)

            ptz_node = LaunchFile.Node(
                name='ptz_action_server_node',
                namespace=image_ns,
                package='clearpath_generator_gz',
                executable='ptz_controller_node',
                parameters=[
                    {'use_sim_time': True},
                    {'camera_name': self.name},
                ],
                remappings=[
                    ('image_in', image_ns + '/_/image_raw'),
                    ('image_out', image_ns + '/color/image'),
                    ('cmd/velocity', image_ns + '/cmd/velocity'),
                    ('joint_states', '/' + self._robot_namespace + '/platform/joint_states'),
                    ('cmd_pan_vel', cmd_ns + '/_/cmd_pan_vel'),
                    ('cmd_tilt_vel', cmd_ns + '/_/cmd_tilt_vel'),
                ],
            )
            self.extra_gz_nodes.append(ptz_node)

    def generate(self):
        sensor_writer = LaunchWriter(self.launch_file)
        # Add sensor bridge and tf nodes
        sensor_writer.add(self.gz_bridge_node)
        sensor_writer.add(self.static_tf_node)
        sensor_writer.add(self.prefix_launch_arg)
        for node in self.extra_gz_nodes:
            sensor_writer.add(node)
        # Generate sensor launch file
        sensor_writer.generate_file()

    @property
    def namespace(self) -> str:
        """Return sensor namespace."""
        if self._robot_namespace in ('', '/'):
            return f'{self.TOPIC_NAMESPACE}'
        else:
            return f'{self._robot_namespace}/{self.TOPIC_NAMESPACE}'

    @property
    def name(self) -> str:
        """Return sensor name."""
        return self.sensor.name

    @property
    def robot_name(self) -> str:
        """Return robot name."""
        if self._robot_namespace in ('', '/'):
            return 'robot'
        else:
            return self._robot_namespace + '/robot'

    @property
    def model(self) -> str:
        """Return sensor model."""
        return self.sensor.SENSOR_MODEL

    def get_gz_bridge_arg(self, suffix: str, gz_to_ros: str) -> list:
        return [
          LaunchFile.Variable('prefix'),
          self.sensor.get_name() + '/' + suffix + gz_to_ros
        ]

    def get_gz_bridge_remap(self, suffix: str, topic: str) -> tuple:
        return (
          [
            LaunchFile.Variable('prefix'),
            self.sensor.get_name() + '/' + suffix
          ],
          topic
        )
