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
from clearpath_generator_common.launch.generator import LaunchGenerator
from clearpath_generator_common.launch.writer import LaunchWriter
from clearpath_generator_gz.launch import platforms  # noqa: F401
from clearpath_generator_gz.launch.platform import PlatformLaunch
from clearpath_generator_gz.launch.sensors import SensorLaunch


class GzLaunchGenerator(LaunchGenerator):
    """Concrete launch generator that emits the Gazebo-side service launch files."""

    def __init__(self, setup_path: str = '/etc/clearpath/') -> None:
        """Initialize the generator and force `use_sim_time` on the shared platform launch."""
        super().__init__(setup_path)
        for i, arg in enumerate(self.platform_launch_file.args):
            if arg[0] == 'use_sim_time':
                self.platform_launch_file.args[i] = ('use_sim_time', 'true')
        self.platform_launch_file.args.append(
            ('use_manipulation_controllers', 'true')
        )

        if self.namespace in ('', '/'):
            self.robot_name = 'robot'
        else:
            self.robot_name = self.namespace + '/robot'

    def generate_sensors(self) -> None:
        sensors_service_launch_writer = LaunchWriter(self.sensors_service_launch_file)
        sensors = self.clearpath_config.sensors.get_all_sensors()

        for sensor in sensors:
            if sensor.get_launch_enabled():
                sensor_launch = SensorLaunch(
                        sensor,
                        self.namespace,
                        self.sensors_launch_path,
                        self.sensors_params_path)
                sensor_launch.generate()
                # Add sensor to top level sensors launch file
                sensors_service_launch_writer.add_launch_file(sensor_launch.launch_file)

        sensors_service_launch_writer.generate_file()

    def generate_platform(self) -> None:
        """Generate the Gazebo platform service launch file via the PlatformLaunch registry."""
        platform_service_launch_writer = LaunchWriter(self.platform_service_launch_file)
        platform_service_launch_writer.add_launch_file(self.platform_launch_file)

        try:
            platform_launch_cls = PlatformLaunch.get(self.platform_model)
        except KeyError:
            platform_service_launch_writer.generate_file()
            return

        platform_launch = platform_launch_cls(
            self.namespace,
            self.robot_name,
            self.platform_params_path,
            self.sensors_params_path,
        )
        for component in platform_launch.get_components():
            platform_service_launch_writer.add(component)

        platform_service_launch_writer.generate_file()

    def generate_manipulators(self) -> None:
        manipulators_service_launch_writer = LaunchWriter(self.manipulators_service_launch_file)
        manipulators_service_launch_writer.generate_file()
