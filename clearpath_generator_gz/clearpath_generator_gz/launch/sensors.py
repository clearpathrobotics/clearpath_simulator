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

from clearpath_config.sensors.base import BaseSensor
from clearpath_config.sensors.lidars_2d import BaseLidar2D, HokuyoUST10, SickLMS1XX
from clearpath_config.sensors.lidars_3d import BaseLidar3D, VelodyneLidar
from clearpath_config.sensors.cameras import BaseCamera, IntelRealsense
from clearpath_config.sensors.imu import BaseIMU, Microstrain
from clearpath_config.sensors.gps import BaseGPS, SwiftNavDuro

from clearpath_generator_common.common import LaunchFile, ParamFile


class SensorLaunch():
    class BaseLaunch():
        TOPIC_NAMESPACE = 'platform/sensors/'

        # Launch arguments
        PARAMETERS = 'parameters'
        NAMESPACE = 'namespace'

        # gz to ros bridge parameters
        GZ_TO_ROS_LASERSCAN = '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        GZ_TO_ROS_POINTCLOUD = '@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        GZ_TO_ROS_IMAGE = '@sensor_msgs/msg/Image[ignition.msgs.Image'
        GZ_TO_ROS_CAMERA_INFO = '@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        GZ_TO_ROS_IMU = '@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        GZ_TO_ROS_NAVSAT = '@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'

        def __init__(self,
                     sensor: BaseSensor,
                     namespace: str,
                     launch_path: str,
                     param_path: str) -> None:
            self.sensor = sensor
            self.namespace = namespace
            self.parameters = ParamFile(self.get_name(), path=param_path)

            if self.namespace in ('', '/'):
                self.robot_name = 'robot'
            else:
                self.robot_name = self.namespace + '/robot'

            # Generated
            self.sensor_launch_file = LaunchFile(
                self.get_name(),
                path=launch_path)

            self.static_tf_node = LaunchFile.get_static_tf_node(
                name=sensor.get_name(),
                namespace=self.namespace,
                parent_link=sensor.get_name() + '_link',
                child_link=self.robot_name + '/base_link/' + sensor.get_name(),
                use_sim_time=True
            )

            self.gz_bridge_node = LaunchFile.Node(
                name=sensor.get_name() + '_gz_bridge',
                namespace=self.namespace,
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'use_sim_time': True}]
            )

        def get_launch_file(self) -> LaunchFile:
            return self.sensor_launch_file

        def get_name(self) -> str:
            return self.sensor.get_name()

        def get_model(self) -> str:
            return self.sensor.SENSOR_MODEL

        def get_namespace(self) -> str:
            return self.TOPIC_NAMESPACE + self.sensor.get_name()

        def get_static_tf_node(self) -> LaunchFile.Node:
            return self.static_tf_node

        def get_gz_bridge_node(self) -> LaunchFile.Node:
            return self.gz_bridge_node

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

    class Lidar2dLaunch(BaseLaunch):
        def __init__(self, sensor: BaseLidar2D, namespace: str, launch_path: str, param_path: str) -> None:
            super().__init__(sensor, namespace, launch_path, param_path)

            self.gz_bridge_node.arguments = [
              self.get_gz_bridge_arg('scan', self.GZ_TO_ROS_LASERSCAN)
            ]

            self.gz_bridge_node.remappings = [
              self.get_gz_bridge_remap('scan', 'platform/sensors/' + sensor.get_topic())
            ]

    class Lidar3dLaunch(BaseLaunch):
        def __init__(self, sensor: BaseLidar3D, namespace: str, launch_path: str, param_path: str) -> None:
            super().__init__(sensor, namespace, launch_path, param_path)

            self.gz_bridge_node.arguments = [
              self.get_gz_bridge_arg('scan/points', self.GZ_TO_ROS_POINTCLOUD)
            ]

            self.gz_bridge_node.remappings = [
              self.get_gz_bridge_remap('scan/points', 'platform/sensors/' + sensor.get_topic())
            ]

    class ImuLaunch(BaseLaunch):
        def __init__(self, sensor: BaseIMU, namespace: str, launch_path: str, param_path: str) -> None:
            super().__init__(sensor, namespace, launch_path, param_path)

            self.gz_bridge_node.arguments = [
              self.get_gz_bridge_arg('imu', self.GZ_TO_ROS_IMU)
            ]

            self.gz_bridge_node.remappings = [
              self.get_gz_bridge_remap('imu', 'platform/sensors/' + sensor.get_name() + '/data')
            ]

    class CameraLaunch(BaseLaunch):
        def __init__(self, sensor: BaseCamera, namespace: str, launch_path: str, param_path: str) -> None:
            super().__init__(sensor, namespace, launch_path, param_path)

            self.gz_bridge_node.arguments = [
              self.get_gz_bridge_arg('camera_info', self.GZ_TO_ROS_CAMERA_INFO),
              self.get_gz_bridge_arg('image', self.GZ_TO_ROS_IMAGE)
            ]

            self.gz_bridge_node.remappings = [
              self.get_gz_bridge_remap('camera_info', 'platform/sensors/' + sensor.get_name() + '/color/camera_info'),
              self.get_gz_bridge_remap('image', 'platform/sensors/' + sensor.get_name() + '/color/image'),
            ]

    class IntelRealsenseLaunch(CameraLaunch):
        def __init__(self, sensor: IntelRealsense, namespace: str, launch_path: str, param_path: str) -> None:
            super().__init__(sensor, namespace, launch_path, param_path)

            if sensor.get_pointcloud_enabled():
                self.gz_bridge_node.arguments.append(
                  self.get_gz_bridge_arg('points', self.GZ_TO_ROS_POINTCLOUD)
                )

                self.gz_bridge_node.remappings.append(
                  self.get_gz_bridge_remap(
                    'points',
                    'platform/sensors/' + sensor.get_name() + '/points')
                )

            if sensor.get_depth_enabled():
                self.gz_bridge_node.arguments.append(
                  self.get_gz_bridge_arg('depth_image', self.GZ_TO_ROS_IMAGE)
                )

                self.gz_bridge_node.remappings.append(
                  self.get_gz_bridge_remap(
                    'depth_image',
                    'platform/sensors/' + sensor.get_name() + '/depth/image')
                )

    class GPSLaunch(BaseLaunch):
        def __init__(self, sensor: BaseGPS, namespace: str, launch_path: str, param_path: str) -> None:
            super().__init__(sensor, namespace, launch_path, param_path)

            self.gz_bridge_node.arguments = [
              self.get_gz_bridge_arg('navsat', self.GZ_TO_ROS_NAVSAT)
            ]

            self.gz_bridge_node.remappings = [
              self.get_gz_bridge_remap(
                'navsat',
                'platform/sensors/' + sensor.get_name() + '/fix')
            ]

    MODEL = {
        HokuyoUST10.SENSOR_MODEL: Lidar2dLaunch,
        SickLMS1XX.SENSOR_MODEL: Lidar2dLaunch,
        IntelRealsense.SENSOR_MODEL: IntelRealsenseLaunch,
        Microstrain.SENSOR_MODEL: ImuLaunch,
        VelodyneLidar.SENSOR_MODEL: Lidar3dLaunch,
        SwiftNavDuro.SENSOR_MODEL: GPSLaunch
    }

    def __new__(cls, sensor: BaseSensor, namespace: str, launch_path: str, param_path: str) -> BaseLaunch:
        return SensorLaunch.MODEL[sensor.SENSOR_MODEL](sensor, namespace, launch_path, param_path)
